#!/usr/bin/env python3
"""
VisionPickPlaceNode — Unified Pick & Place Action Server
===========================================================
pick과 place를 하나의 노드에서 관리. Serial port 충돌 방지.

Roles:
  1. /vision_pick action server
  2. /vision_place action server
  3. MyCobot280 serial 제어 (단일 instance)

Action:
  /vision_pick  [VisionPick]
  /vision_place [VisionPlace]
"""

import os
import time
import threading
from typing import Optional

import requests
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from jetcobot_vision_msgs.action import VisionPick, VisionPlace
from jetcobot_vision_msgs.msg import PickPoint
from jetcobot_vision_msgs.srv import UpdatePose

try:
    from pymycobot import MyCobot280 as _MC280
    _MC_OK = True
except ImportError:
    _MC_OK = False

_PICK_SPEED    = 30
_PLACE_SPEED   = 30
_GRIPPER_OPEN  = 100
_GRIPPER_CLOSE = 0
_MAX_MOVE_WAIT = 30.0


def _load_profiles() -> dict:
    try:
        pkg_dir = get_package_share_directory("jetcobot_vision")
        path = os.path.join(pkg_dir, "config", "pick_place_profiles.yaml")
    except Exception:
        path = os.path.join(os.path.dirname(__file__), "..", "config", "pick_place_profiles.yaml")
    with open(path) as f:
        return yaml.safe_load(f)


_PROFILES: dict[str, dict] = _load_profiles()


class VisionPickPlaceNode(Node):

    def __init__(self) -> None:
        super().__init__("vision_pick_place_node")

        self._cb_group = ReentrantCallbackGroup()

        # MyCobot 초기화 (단일 instance)
        self.declare_parameter("port",                "/dev/ttyJETCOBOT")
        self.declare_parameter("baud",                1_000_000)
        self.declare_parameter("grasp_roll",          -178.81)
        self.declare_parameter("grasp_pitch",           0.94)
        self.declare_parameter("grasp_yaw_offset",      0.0)
        self.declare_parameter("tcp_offset",           [0.0, 20.0, 100.0])
        self.declare_parameter("pick_z_offset_mm",     20.0)
        self.declare_parameter("detect_timeout_sec",   10.0)
        self.declare_parameter("coord_topic",          "/coord_transform_node/pick_point")
        self.declare_parameter("coord_enable_service", "/coord_transform_node/enable")
        self.declare_parameter("cv_detect_server_url", "http://192.168.1.4:8000")

        port                      = self.get_parameter("port").value
        baud                      = self.get_parameter("baud").value
        self._grasp_roll          = self.get_parameter("grasp_roll").value
        self._grasp_pitch         = self.get_parameter("grasp_pitch").value
        self._grasp_yaw_off       = self.get_parameter("grasp_yaw_offset").value
        self._pick_z_off_mm       = self.get_parameter("pick_z_offset_mm").value
        self._tcp_offset          = list(self.get_parameter("tcp_offset").value)
        self._detect_timeout      = self.get_parameter("detect_timeout_sec").value
        coord_topic               = self.get_parameter("coord_topic").value
        coord_enable_service      = self.get_parameter("coord_enable_service").value
        self._cv_server_url       = self.get_parameter("cv_detect_server_url").value

        self._mc = None
        if _MC_OK:
            try:
                self.get_logger().info(f"MyCobot 연결 중: {port} @ {baud}")
                self._mc = _MC280(port, baud)
                time.sleep(0.5)
                self.get_logger().info("MyCobot 연결 완료")
                self.get_logger().info("초기 자세로 이동: [0,0,0,0,0,0]")
                self._mc.send_angles([0, 0, 0, 0, 0, 0], _PICK_SPEED)
            except Exception as exc:
                self.get_logger().error(f"MyCobot 연결 실패: {exc}")
                self._mc = None

        # PickPoint 구독 (pick/place 공용)
        self._pick_points: list[PickPoint] = []
        self._pick_lock = threading.Lock()

        self.create_subscription(
            PickPoint, coord_topic, self._on_pick_point, 10,
            callback_group=self._cb_group,
        )

        # Coord Transform 서비스 클라이언트
        self._coord_enable_client = self.create_client(
            SetBool, coord_enable_service, callback_group=self._cb_group
        )
        self._set_params_client = self.create_client(
            SetParameters, "/coord_transform_node/set_parameters", callback_group=self._cb_group
        )
        self._update_pose_client = self.create_client(
            UpdatePose, "/coord_transform_node/update_pose", callback_group=self._cb_group
        )

        # Action Servers
        self._pick_server = ActionServer(
            self, VisionPick, "/vision_pick",
            execute_callback=self._execute_pick,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self._place_server = ActionServer(
            self, VisionPlace, "/vision_place",
            execute_callback=self._execute_place,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info("VisionPickPlaceNode 시작 완료")

    def _on_pick_point(self, msg: PickPoint) -> None:
        with self._pick_lock:
            self._pick_points.append(msg)

    def _goal_cb(self, goal_request) -> GoalResponse:
        loc = goal_request.location
        self.get_logger().info(f"Goal 수락: location={loc}")
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _) -> CancelResponse:
        self.get_logger().info("취소 요청 수락")
        return CancelResponse.ACCEPT

    async def _execute_pick(self, goal_handle) -> VisionPick.Result:
        """Pick Action 실행."""
        location = goal_handle.request.location
        box_index = goal_handle.request.box_index if hasattr(goal_handle.request, 'box_index') else -1
        fb = VisionPick.Feedback()
        res = VisionPick.Result()

        self.get_logger().info(f"[Pick] 실행 시작  location={location}  box_index={box_index}")

        # Phase 1: moving
        self._fb(goal_handle, fb, "moving", 0.10)

        profile = self._load_profile(location)
        if profile is None:
            return self._abort(goal_handle, res, f"알 수 없는 location: '{location}'")

        # 비전 기반 pick
        self._send_cv_config(profile)
        self._set_coord_transform(False)

        if not self._move_to_observe(location, profile):
            self._set_coord_transform(False)
            return self._abort(goal_handle, res, f"observe_pose 이동 실패: {location}")

        if self._mc is not None:
            actual = self._mc.get_coords()
            if actual and len(actual) == 6:
                self._update_coord_transform_pose(actual)

        self._set_coord_transform(True)

        # Phase 2: detecting
        self._fb(goal_handle, fb, "detecting", 0.30)
        pick_pt = self._wait_for_point(box_index)
        if pick_pt is None:
            self._set_coord_transform(False)
            return self._abort(goal_handle, res, "검출 타임아웃")

        self._set_coord_transform(False)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success, res.message = False, "Goal 취소됨"
            return res

        # Phase 3: transforming
        self._fb(goal_handle, fb, "transforming", 0.50)
        x_mm = pick_pt.x * 1000.0
        y_mm = pick_pt.y * 1000.0
        z_mm = pick_pt.z * 1000.0
        yaw_deg = pick_pt.yaw_deg

        offset = profile.get("pick_offset_mm", [0.0, 0.0, 0.0])
        x_mm += offset[0]
        y_mm += offset[1]
        z_mm += offset[2]

        self.get_logger().info(
            f"[Pick] 픽업 좌표: x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm  yaw={yaw_deg:.1f} deg"
        )
        if any(o != 0.0 for o in offset):
            self.get_logger().info(f"[Pick] 보정값 적용: dx={offset[0]:.1f} dy={offset[1]:.1f} dz={offset[2]:.1f} mm")

        # Phase 4: picking
        self._fb(goal_handle, fb, "picking", 0.70)
        if not self._do_pick(x_mm, y_mm, z_mm, yaw_deg, profile):
            return self._abort(goal_handle, res, "픽업 동작 실패")

        # Phase 5: done
        self._fb(goal_handle, fb, "done", 1.00)
        res.success = True
        res.message = f"location={location} 픽업 완료"
        res.pick_point_base.x = pick_pt.x
        res.pick_point_base.y = pick_pt.y
        res.pick_point_base.z = pick_pt.z
        goal_handle.succeed()
        return res

    async def _execute_place(self, goal_handle) -> VisionPlace.Result:
        """Place Action 실행."""
        location = goal_handle.request.location
        box_index = goal_handle.request.box_index if hasattr(goal_handle.request, 'box_index') else -1
        fb = VisionPlace.Feedback()
        res = VisionPlace.Result()

        self.get_logger().info(f"[Place] 실행 시작  location={location}  box_index={box_index}")

        # Phase 1: moving
        self._fb(goal_handle, fb, "moving", 0.10)

        profile = self._load_profile(location)
        if profile is None:
            return self._abort(goal_handle, res, f"알 수 없는 location: '{location}'")

        self._send_cv_config(profile)
        self._set_coord_transform(False)

        if not self._move_to_observe(location, profile):
            self._set_coord_transform(False)
            return self._abort(goal_handle, res, f"observe_pose 이동 실패: {location}")

        if self._mc is not None:
            actual = self._mc.get_coords()
            if actual and len(actual) == 6:
                self._update_coord_transform_pose(actual)

        self._set_coord_transform(True)

        # Phase 2: detecting
        self._fb(goal_handle, fb, "detecting", 0.30)
        place_pt = self._wait_for_point(box_index)
        if place_pt is None:
            self._set_coord_transform(False)
            return self._abort(goal_handle, res, "검출 타임아웃")

        self._set_coord_transform(False)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success, res.message = False, "Goal 취소됨"
            return res

        x_mm = place_pt.x * 1000.0
        y_mm = place_pt.y * 1000.0
        z_mm = place_pt.z * 1000.0
        yaw_deg = place_pt.yaw_deg

        offset = profile.get("pick_offset_mm", [0.0, 0.0, 0.0])
        x_mm += offset[0]
        y_mm += offset[1]
        z_mm += offset[2]

        self.get_logger().info(
            f"[Place] place 좌표: x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm  yaw={yaw_deg:.1f} deg"
        )
        if any(o != 0.0 for o in offset):
            self.get_logger().info(f"[Place] 보정값 적용: dx={offset[0]:.1f} dy={offset[1]:.1f} dz={offset[2]:.1f} mm")

        # Phase 3: placing
        self._fb(goal_handle, fb, "placing", 0.70)
        if not self._do_place(x_mm, y_mm, z_mm, yaw_deg, profile):
            return self._abort(goal_handle, res, "place 동작 실패")

        # Phase 4: done
        self._fb(goal_handle, fb, "done", 1.00)
        res.success = True
        res.message = f"location={location} place 완료"
        res.place_point_base.x = place_pt.x
        res.place_point_base.y = place_pt.y
        res.place_point_base.z = place_pt.z
        goal_handle.succeed()
        return res

    # ── 헬퍼 메서드 ──────────────────────────────────────────────────────────

    def _load_profile(self, location: str) -> Optional[dict]:
        return _PROFILES.get(location)

    def _fb(self, goal_handle, fb, phase: str, progress: float) -> None:
        fb.phase = phase
        fb.progress = progress
        goal_handle.publish_feedback(fb)

    def _abort(self, goal_handle, res: VisionPick.Result, msg: str) -> VisionPick.Result:
        self.get_logger().error(msg)
        res.success = False
        res.message = msg
        goal_handle.abort()
        return res

    def _set_coord_transform(self, enable: bool) -> None:
        if not self._coord_enable_client.service_is_ready():
            self.get_logger().warn("coord_enable 서비스 미준비 — 스킵")
            return
        req = SetBool.Request()
        req.data = enable
        self._coord_enable_client.call(req)

    def _send_cv_config(self, profile: dict) -> None:
        if "z_surface_mm" in profile and profile["z_surface_mm"] is not None:
            z = float(profile["z_surface_mm"])
            try:
                if self._set_params_client.service_is_ready():
                    req = SetParameters.Request()
                    p = Parameter()
                    p.name = "z_surface_mm"
                    p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=z)
                    req.parameters = [p]
                    self._set_params_client.call(req)
            except Exception as exc:
                self.get_logger().warn(f"z_surface_mm 업데이트 실패: {exc}")

        cfg = {}
        for key in ("hsv_lower", "hsv_upper", "min_w", "max_w", "min_h", "max_h"):
            if key in profile and profile[key] is not None:
                cfg[key] = profile[key]
        if cfg:
            try:
                url = self._cv_server_url.rstrip("/") + "/config"
                requests.post(url, json=cfg, timeout=2.0)
            except Exception as exc:
                self.get_logger().warn(f"cv_detect_server 전송 실패: {exc}")

    def _move_to_observe(self, location: str, profile: dict) -> bool:
        if self._mc is None:
            return False
        obs = profile.get("observe_pose")
        if not obs or len(obs) != 6:
            return False
        try:
            self._mc.send_coords(obs, _PICK_SPEED)
            deadline = time.monotonic() + _MAX_MOVE_WAIT
            while time.monotonic() < deadline:
                is_moving = self._mc.is_moving()
                if not is_moving:
                    return True
                time.sleep(0.2)
            return False
        except Exception as exc:
            self.get_logger().error(f"observe_pose 이동 실패: {exc}")
            return False

    def _update_coord_transform_pose(self, actual: list) -> None:
        if not self._update_pose_client.service_is_ready():
            return
        try:
            req = UpdatePose.Request()
            req.coords = actual[:6]
            self._update_pose_client.call(req)
        except Exception as exc:
            self.get_logger().warn(f"pose 업데이트 실패: {exc}")

    def _wait_for_point(self, box_index: int = -1) -> Optional[PickPoint]:
        with self._pick_lock:
            self._pick_points.clear()
        deadline = time.monotonic() + self._detect_timeout
        while time.monotonic() < deadline:
            with self._pick_lock:
                if self._pick_points:
                    if box_index < 0:
                        selected = max(self._pick_points, key=lambda p: p.confidence)
                    else:
                        if box_index < len(self._pick_points):
                            selected = self._pick_points[box_index]
                        else:
                            self.get_logger().warn(f"box_index {box_index} 범위 초과")
                            return None
                    self.get_logger().info(f"선택된 상자: index={selected.box_index} conf={selected.confidence:.2f}")
                    return selected
            time.sleep(0.05)
        self.get_logger().warn(f"검출 타임아웃 ({self._detect_timeout:.1f}s)")
        return None

    def _do_pick(self, x_mm: float, y_mm: float, z_mm: float, yaw_deg: float, profile: Optional[dict] = None) -> bool:
        if self._mc is None:
            return False
        roll = profile.get("grasp_roll", self._grasp_roll) if profile else self._grasp_roll
        pitch = profile.get("grasp_pitch", self._grasp_pitch) if profile else self._grasp_pitch
        yaw_offset = profile.get("grasp_yaw_offset", self._grasp_yaw_off) if profile else self._grasp_yaw_off
        z_offset = profile.get("pick_z_offset_mm", self._pick_z_off_mm) if profile else self._pick_z_off_mm

        rz = yaw_deg + yaw_offset
        ox, oy, oz = self._tcp_offset

        try:
            # Approach
            approach = [x_mm + ox, y_mm + oy, z_mm + oz + z_offset, roll, pitch, rz]
            self._mc.send_coords(approach, _PICK_SPEED)
            self._wait_moving()

            # Grasp
            self._mc.set_gripper_value(_GRIPPER_CLOSE, 30)
            time.sleep(1.0)

            # Retreat
            retreat = [x_mm + ox, y_mm + oy, z_mm + oz + z_offset + 50, roll, pitch, rz]
            self._mc.send_coords(retreat, _PICK_SPEED)
            self._wait_moving()

            return True
        except Exception as exc:
            self.get_logger().error(f"pick 동작 실패: {exc}")
            return False

    def _do_place(self, x_mm: float, y_mm: float, z_mm: float, yaw_deg: float, profile: Optional[dict] = None) -> bool:
        if self._mc is None:
            return False
        roll = profile.get("grasp_roll", self._grasp_roll) if profile else self._grasp_roll
        pitch = profile.get("grasp_pitch", self._grasp_pitch) if profile else self._grasp_pitch
        yaw_offset = profile.get("grasp_yaw_offset", self._grasp_yaw_off) if profile else self._grasp_yaw_off
        z_offset = profile.get("pick_z_offset_mm", self._pick_z_off_mm) if profile else self._pick_z_off_mm

        rz = yaw_deg + yaw_offset
        ox, oy, oz = self._tcp_offset

        try:
            # Approach
            approach = [x_mm + ox, y_mm + oy, z_mm + oz + z_offset, roll, pitch, rz]
            self._mc.send_coords(approach, _PLACE_SPEED)
            self._wait_moving()

            # Release
            self._mc.set_gripper_value(_GRIPPER_OPEN, 30)
            time.sleep(1.0)

            # Retreat
            retreat = [x_mm + ox, y_mm + oy, z_mm + oz + z_offset + 50, roll, pitch, rz]
            self._mc.send_coords(retreat, _PLACE_SPEED)
            self._wait_moving()

            return True
        except Exception as exc:
            self.get_logger().error(f"place 동작 실패: {exc}")
            return False

    def _wait_moving(self) -> None:
        if self._mc is None:
            return
        deadline = time.monotonic() + _MAX_MOVE_WAIT
        while time.monotonic() < deadline:
            if not self._mc.is_moving():
                return
            time.sleep(0.1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionPickPlaceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
