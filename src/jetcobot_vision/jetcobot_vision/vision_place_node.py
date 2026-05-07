#!/usr/bin/env python3
"""
VisionPlaceNode — Action Server
=================================
goal.location → observe → detect → place

Phase 1 "moving"    : location 프로파일 로드 → cv_detect_server /config 전송
                      → coord_transform 비활성화 → observe_pose 이동
                      → get_coords() → update_pose → coord_transform 활성화
Phase 2 "detecting" : pick_point 토픽 대기 → coord_transform 비활성화
Phase 3 "placing"   : 접근 → 하강 → 그리퍼 오픈 → 리프트
Phase 4 "done"      : Result 반환

fixed_coords 프로파일 (warehouse_place 등):
  observe_pose가 None이면 비전 스킵, fixed_coords로 직접 place

Action  /vision_place  [jetcobot_vision_msgs/action/VisionPlace]
"""

import os
import time
import threading
from typing import Optional

import requests
import yaml
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_share_directory

from jetcobot_vision_msgs.action import VisionPlace
from jetcobot_vision_msgs.msg import PickPoint
from jetcobot_vision_msgs.srv import UpdatePose

try:
    from pymycobot import MyCobot280 as _MC280
    _MC_OK = True
except ImportError:
    _MC_OK = False

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


class VisionPlaceNode(Node):

    def __init__(self) -> None:
        super().__init__("vision_place_node")

        self._cb_group = ReentrantCallbackGroup()

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

        port                 = self.get_parameter("port").value
        baud                 = self.get_parameter("baud").value
        self._grasp_roll     = self.get_parameter("grasp_roll").value
        self._grasp_pitch    = self.get_parameter("grasp_pitch").value
        self._grasp_yaw_off  = self.get_parameter("grasp_yaw_offset").value
        self._tcp_offset     = list(self.get_parameter("tcp_offset").value)
        self._pick_z_off_mm  = self.get_parameter("pick_z_offset_mm").value
        self._detect_timeout = self.get_parameter("detect_timeout_sec").value
        coord_topic          = self.get_parameter("coord_topic").value
        coord_enable_service = self.get_parameter("coord_enable_service").value
        self._cv_server_url  = self.get_parameter("cv_detect_server_url").value

        if _MC_OK:
            try:
                self.get_logger().info(f"MyCobot 연결 중: {port} @ {baud}")
                self._mc = _MC280(port, baud)
                time.sleep(0.5)
                self.get_logger().info("MyCobot 연결 완료")
            except Exception as exc:
                self._mc = None
                self.get_logger().warn(f"MyCobot 연결 실패 — 모의 동작 모드: {exc}")
        else:
            self._mc = None
            self.get_logger().warn("pymycobot 미설치 — 모의 동작 모드")

        self._latest_pick: Optional[PickPoint] = None
        self._pick_lock = threading.Lock()

        self.create_subscription(
            PickPoint, coord_topic, self._on_pick_point, 10,
            callback_group=self._cb_group,
        )

        self._coord_enable_client = self.create_client(
            SetBool, coord_enable_service, callback_group=self._cb_group
        )
        self._update_pose_client = self.create_client(
            UpdatePose, "/coord_transform_node/update_pose", callback_group=self._cb_group
        )

        self._action_server = ActionServer(
            self, VisionPlace, "vision_place",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info("VisionPlaceNode Action Server 대기 중")

    # ── 헬퍼 ─────────────────────────────────────────────────────────────────

    def _set_coord_transform(self, enable: bool) -> None:
        if not self._coord_enable_client.service_is_ready():
            self.get_logger().warn("coord_enable 서비스 미준비 — 스킵")
            return
        req = SetBool.Request()
        req.data = enable
        self._coord_enable_client.call(req)
        self.get_logger().info(f"coord_transform {'활성화' if enable else '비활성화'}")

    def _update_coord_transform_pose(self, coords: list) -> None:
        if not self._update_pose_client.service_is_ready():
            self.get_logger().warn("update_pose 서비스 미준비 — yaml 고정값 사용")
            return
        req = UpdatePose.Request()
        req.coords = [float(c) for c in coords]
        self._update_pose_client.call(req)
        self.get_logger().info(f"실제 EE 좌표 전달: {[round(c, 2) for c in coords]}")

    def _send_cv_config(self, profile: dict) -> None:
        # HSV/W/H → cv_detect_server (HTTP POST /config)
        cfg = {}
        for key in ("hsv_lower", "hsv_upper", "min_w", "max_w", "min_h", "max_h",
                    "min_area", "max_area", "morph_k"):
            if key in profile and profile[key] is not None:
                cfg[key] = profile[key]
        if cfg:
            try:
                url = self._cv_server_url.rstrip("/") + "/config"
                resp = requests.post(url, json=cfg, timeout=2.0)
                self.get_logger().info(f"cv_detect_server /config 전송: {cfg}")
            except Exception as exc:
                self.get_logger().warn(f"cv_detect_server /config 전송 실패: {exc}")

    def _load_profile(self, location: str) -> Optional[dict]:
        return _PROFILES.get(location)

    def _on_pick_point(self, msg: PickPoint) -> None:
        with self._pick_lock:
            self._latest_pick = msg

    def _goal_cb(self, goal_request) -> GoalResponse:
        self.get_logger().info(f"Goal 수락: location={goal_request.location}")
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _) -> CancelResponse:
        self.get_logger().info("취소 요청 수락")
        return CancelResponse.ACCEPT

    # ── Action 실행 ───────────────────────────────────────────────────────────

    async def _execute_cb(self, goal_handle) -> VisionPlace.Result:
        location = goal_handle.request.location
        fb  = VisionPlace.Feedback()
        res = VisionPlace.Result()

        self.get_logger().info(f"[VisionPlace] 실행 시작  location={location}")

        profile = self._load_profile(location)
        if profile is None:
            return self._abort(goal_handle, res, f"알 수 없는 location: '{location}'")

        # ── Phase 1: moving ──────────────────────────────────────────────────
        self._fb(goal_handle, fb, "moving", 0.10)
        self._send_cv_config(profile)

        # fixed_coords 프로파일 (비전 없음) 처리
        if profile.get("observe_pose") is None:
            fixed = profile.get("fixed_coords")
            if not fixed:
                return self._abort(goal_handle, res, f"observe_pose/fixed_coords 미설정: {location}")
            self._fb(goal_handle, fb, "placing", 0.50)
            if not self._do_place_fixed(fixed, profile):
                return self._abort(goal_handle, res, "fixed place 동작 실패")
            self._fb(goal_handle, fb, "done", 1.00)
            res.success = True
            res.message = f"location={location} place 완료 (fixed)"
            goal_handle.succeed()
            return res

        # 비전 기반 place
        self._set_coord_transform(False)
        if not self._move_to_observe(profile):
            self._set_coord_transform(False)
            return self._abort(goal_handle, res, "observe_pose 이동 실패")

        if self._mc is not None:
            actual = self._mc.get_coords()
            if actual and len(actual) == 6:
                self._update_coord_transform_pose(actual)

        self._set_coord_transform(True)

        # ── Phase 2: detecting ───────────────────────────────────────────────
        self._fb(goal_handle, fb, "detecting", 0.40)
        place_pt = self._wait_for_point()
        if place_pt is None:
            self._set_coord_transform(False)
            return self._abort(goal_handle, res, "검출 타임아웃")

        self._set_coord_transform(False)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success, res.message = False, "Goal 취소됨"
            return res

        x_mm    = place_pt.x * 1000.0
        y_mm    = place_pt.y * 1000.0
        z_mm    = place_pt.z * 1000.0
        yaw_deg = place_pt.yaw_deg

        # Location별 pick_offset 적용
        offset = profile.get("pick_offset_mm", [0.0, 0.0, 0.0])
        x_mm += offset[0]
        y_mm += offset[1]
        z_mm += offset[2]

        self.get_logger().info(
            f"[VisionPlace] place 좌표: x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm  yaw={yaw_deg:.1f} deg"
        )
        if any(o != 0.0 for o in offset):
            self.get_logger().info(f"[VisionPlace] 보정값 적용: dx={offset[0]:.1f} dy={offset[1]:.1f} dz={offset[2]:.1f} mm")

        # ── Phase 3: placing ─────────────────────────────────────────────────
        self._fb(goal_handle, fb, "placing", 0.70)
        if not self._do_place(x_mm, y_mm, z_mm, yaw_deg, profile):
            return self._abort(goal_handle, res, "place 동작 실패")

        # ── Phase 4: done ────────────────────────────────────────────────────
        self._fb(goal_handle, fb, "done", 1.00)
        res.success = True
        res.message = f"location={location} place 완료"
        res.place_point_base.x = place_pt.x
        res.place_point_base.y = place_pt.y
        res.place_point_base.z = place_pt.z
        goal_handle.succeed()
        self.get_logger().info("[VisionPlace] 완료")
        return res

    @staticmethod
    def _fb(gh, fb: VisionPlace.Feedback, phase: str, progress: float) -> None:
        fb.phase, fb.progress = phase, progress
        gh.publish_feedback(fb)

    @staticmethod
    def _abort(gh, res: VisionPlace.Result, msg: str) -> VisionPlace.Result:
        res.success, res.message = False, msg
        gh.abort()
        return res

    # ── 이동 ─────────────────────────────────────────────────────────────────

    def _move_to_observe(self, profile: dict) -> bool:
        pose = list(profile["observe_pose"])
        if self._mc is None:
            self.get_logger().info(f"[모의] observe_pose 이동: {pose}")
            time.sleep(1.0)
            return True
        self._mc.send_coords(pose, _PLACE_SPEED, 0)
        self._wait_move(settle=1.0)
        return True

    def _wait_for_point(self) -> Optional[PickPoint]:
        with self._pick_lock:
            self._latest_pick = None
        deadline = time.monotonic() + self._detect_timeout
        while time.monotonic() < deadline:
            with self._pick_lock:
                if self._latest_pick is not None:
                    return self._latest_pick
            time.sleep(0.05)
        self.get_logger().warn(f"검출 타임아웃 ({self._detect_timeout:.1f}s)")
        return None

    def _do_place(self, x_mm: float, y_mm: float, z_mm: float, yaw_deg: float,
                  profile: Optional[dict] = None) -> bool:
        roll       = profile.get("grasp_roll",       self._grasp_roll)    if profile else self._grasp_roll
        pitch      = profile.get("grasp_pitch",      self._grasp_pitch)   if profile else self._grasp_pitch
        yaw_offset = profile.get("grasp_yaw_offset", self._grasp_yaw_off) if profile else self._grasp_yaw_off
        z_offset   = profile.get("pick_z_offset_mm", self._pick_z_off_mm) if profile else self._pick_z_off_mm
        rx, ry, rz = roll, pitch, yaw_deg + yaw_offset
        ox, oy, oz = self._tcp_offset[0], self._tcp_offset[1], self._tcp_offset[2]
        approach  = [x_mm + ox, y_mm + oy, z_mm + oz + z_offset, rx, ry, rz]
        place_pos = [x_mm + ox, y_mm + oy, z_mm + oz, rx, ry, rz]

        if self._mc is None:
            self.get_logger().info(f"[모의] 접근: {approach}  적재: {place_pos}")
            time.sleep(1.0)
            return True

        self._mc.send_coords(approach, _PLACE_SPEED, 0)
        self._wait_move(settle=1.0)
        self._mc.send_coords(place_pos, _PLACE_SPEED // 2, 0)
        self._wait_move(settle=0.5)
        self._mc.set_gripper_value(_GRIPPER_OPEN, 50)
        time.sleep(1.0)
        self._mc.send_coords(approach, _PLACE_SPEED, 0)
        self._wait_move(settle=1.0)
        return True

    def _do_place_fixed(self, fixed_coords: list, profile: Optional[dict] = None) -> bool:
        """비전 없이 티칭 좌표로 직접 place."""
        if self._mc is None:
            self.get_logger().info(f"[모의] fixed place: {fixed_coords}")
            time.sleep(1.0)
            return True
        z_offset  = profile.get("pick_z_offset_mm", self._pick_z_off_mm) if profile else self._pick_z_off_mm
        x, y, z   = fixed_coords[0], fixed_coords[1], fixed_coords[2]
        rx, ry, rz = fixed_coords[3], fixed_coords[4], fixed_coords[5]
        approach  = [x, y, z + z_offset, rx, ry, rz]
        place_pos = [x, y, z, rx, ry, rz]
        self._mc.send_coords(approach, _PLACE_SPEED, 0)
        self._wait_move(settle=1.0)
        self._mc.send_coords(place_pos, _PLACE_SPEED // 2, 0)
        self._wait_move(settle=0.5)
        self._mc.set_gripper_value(_GRIPPER_OPEN, 50)
        time.sleep(1.0)
        self._mc.send_coords(approach, _PLACE_SPEED, 0)
        self._wait_move(settle=1.0)
        return True

    def _wait_move(self, settle: float = 1.0) -> None:
        time.sleep(settle)
        deadline = time.monotonic() + _MAX_MOVE_WAIT
        while time.monotonic() < deadline:
            try:
                if not self._mc.is_moving():
                    break
            except Exception as exc:
                self.get_logger().warn(f"is_moving() 예외: {exc}")
                break
            time.sleep(0.1)
        else:
            self.get_logger().warn("이동 타임아웃 — 강제 진행")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionPlaceNode()
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
