#!/usr/bin/env python3
"""
VisionPickNode — Action Server
================================
goal.location → observe → detect → transform → pick

Phase 1 "moving"       : location 프로파일 로드 → cv_detect_server /config 전송
                         → coord_transform 활성화 → observe_pose 이동
Phase 2 "detecting"    : pick_point_base 토픽 대기
Phase 3 "transforming" : coord_transform 비활성화 → 좌표 검증 로깅
Phase 4 "picking"      : 접근 → 하강 → 파지 → 리프트
Phase 5 "done"         : Result 반환

Action  /vision_pick  [jetcobot_vision_msgs/action/VisionPick]
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

from jetcobot_vision_msgs.action import VisionPick
from jetcobot_vision_msgs.msg import PickPoint
from jetcobot_vision_msgs.srv import UpdatePose

try:
    from pymycobot import MyCobot280 as _MC280
    _MC_OK = True
except ImportError:
    _MC_OK = False

_PICK_SPEED    = 30
_GRIPPER_OPEN  = 100
_GRIPPER_CLOSE = 0
_MAX_MOVE_WAIT = 30.0

def _load_profiles() -> dict:
    """pick_place_profiles.yaml을 로드. 패키지 share 경로 우선, 없으면 소스 경로 사용."""
    try:
        pkg_dir = get_package_share_directory("jetcobot_vision")
        path = os.path.join(pkg_dir, "config", "pick_place_profiles.yaml")
    except Exception:
        path = os.path.join(os.path.dirname(__file__), "..", "config", "pick_place_profiles.yaml")
    with open(path) as f:
        return yaml.safe_load(f)

_PROFILES: dict[str, dict] = _load_profiles()


class VisionPickNode(Node):

    def __init__(self) -> None:
        super().__init__("vision_pick_node")

        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter("port",                "/dev/ttyJETCOBOT")
        self.declare_parameter("baud",                1_000_000)
        self.declare_parameter("grasp_roll",          -178.81)
        self.declare_parameter("grasp_pitch",           0.94)
        self.declare_parameter("grasp_yaw_offset",      0.0)
        self.declare_parameter("tcp_offset",           [0.0, 20.0, 100.0])
        self.declare_parameter("pick_z_offset_mm",     20.0)
        self.declare_parameter("detect_timeout_sec",   10.0)
        self.declare_parameter("coord_topic",          "/coord_transform_node/pick_point_base")
        self.declare_parameter("cv_server_url",        "http://192.168.1.4:8081")
        self.declare_parameter("coord_enable_service", "/coord_transform_node/enable")

        port                      = self.get_parameter("port").value
        baud                      = self.get_parameter("baud").value
        self._grasp_roll          = self.get_parameter("grasp_roll").value
        self._grasp_pitch         = self.get_parameter("grasp_pitch").value
        self._grasp_yaw_off       = self.get_parameter("grasp_yaw_offset").value
        self._pick_z_off_mm       = self.get_parameter("pick_z_offset_mm").value
        self._tcp_offset          = list(self.get_parameter("tcp_offset").value)
        self._detect_timeout      = self.get_parameter("detect_timeout_sec").value
        coord_topic               = self.get_parameter("coord_topic").value
        self._cv_server_url       = self.get_parameter("cv_server_url").value
        coord_enable_service      = self.get_parameter("coord_enable_service").value

        if _MC_OK:
            try:
                self.get_logger().info(f"MyCobot 연결 중: {port} @ {baud}")
                self._mc = _MC280(port, baud)
                time.sleep(0.5)
                self.get_logger().info("MyCobot 연결 완료")
                self.get_logger().info("초기 자세로 이동: [0,0,0,0,0,0]")
                self._mc.send_angles([0, 0, 0, 0, 0, 0], _PICK_SPEED)
                time.sleep(2.0)
            except Exception as exc:
                self._mc = None
                self.get_logger().warn(f"MyCobot 연결 실패 — 모의 동작 모드: {exc}")
        else:
            self._mc = None
            self.get_logger().warn("pymycobot 미설치 — 모의 동작 모드")

        self._latest_pick: Optional[PickPoint] = None
        self._pick_lock = threading.Lock()

        pick_topic = coord_topic.replace("pick_point_base", "pick_point")
        self.create_subscription(
            PickPoint, pick_topic, self._on_pick_point, 10,
            callback_group=self._cb_group,
        )

        # coord_transform_node 서비스 클라이언트
        self._coord_enable_client = self.create_client(
            SetBool, coord_enable_service, callback_group=self._cb_group
        )
        self._update_pose_client = self.create_client(
            UpdatePose, "/coord_transform_node/update_pose", callback_group=self._cb_group
        )
        self._set_params_client = self.create_client(
            SetParameters, "/coord_transform_node/set_parameters", callback_group=self._cb_group
        )

        self._action_server = ActionServer(
            self, VisionPick, "vision_pick",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info("VisionPickNode Action Server 대기 중")

    # ── 헬퍼: coord_transform T_base2cam 실시간 업데이트 ─────────────────────

    def _update_coord_transform_pose(self, coords: list) -> None:
        if not self._update_pose_client.service_is_ready():
            self.get_logger().warn("update_pose 서비스 미준비 — yaml 고정값 사용")
            return
        req = UpdatePose.Request()
        req.coords = [float(c) for c in coords]
        self._update_pose_client.call(req)
        self.get_logger().info(f"실제 EE 좌표 전달: {[round(c, 2) for c in coords]}")

    # ── 헬퍼: coord_transform enable/disable ─────────────────────────────────

    def _set_coord_transform(self, enable: bool) -> None:
        if not self._coord_enable_client.service_is_ready():
            self.get_logger().warn("coord_enable 서비스 미준비 — 스킵")
            return
        req = SetBool.Request()
        req.data = enable
        self._coord_enable_client.call(req)
        self.get_logger().info(f"coord_transform {'활성화' if enable else '비활성화'}")

    # ── 헬퍼: cv_detect_server 파라미터 전송 ─────────────────────────────────

    def _send_cv_config(self, profile: dict) -> None:
        cfg = {}
        if "hsv_lower" in profile: cfg["hsv_lower"] = list(profile["hsv_lower"])
        if "hsv_upper" in profile: cfg["hsv_upper"] = list(profile["hsv_upper"])
        if "min_w"     in profile: cfg["min_w"]     = int(profile["min_w"])
        if "max_w"     in profile: cfg["max_w"]     = int(profile["max_w"])
        if "min_h"     in profile: cfg["min_h"]     = int(profile["min_h"])
        if "max_h"     in profile: cfg["max_h"]     = int(profile["max_h"])
        if cfg:
            try:
                url = f"{self._cv_server_url}/config"
                requests.post(url, json=cfg, timeout=1.0)
                self.get_logger().info(f"cv_detect_server 파라미터 전송: {cfg}")
            except Exception as exc:
                self.get_logger().warn(f"cv_detect_server /config 전송 실패: {exc}")

        # z_surface_mm을 coord_transform_node 파라미터로 업데이트
        if "z_surface_mm" in profile and profile["z_surface_mm"] is not None:
            z = float(profile["z_surface_mm"])
            try:
                if self._set_params_client.service_is_ready():
                    req = SetParameters.Request()
                    p = Parameter()
                    p.name = "z_surface_mm"
                    p.value = ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE, double_value=z
                    )
                    req.parameters = [p]
                    self._set_params_client.call(req)
                    self.get_logger().info(f"z_surface_mm 업데이트: {z} mm")
            except Exception as exc:
                self.get_logger().warn(f"z_surface_mm 업데이트 실패: {exc}")

    # ── 헬퍼: location 프로파일 로드 ─────────────────────────────────────────

    def _load_profile(self, location: str) -> Optional[dict]:
        """location에 해당하는 파라미터 프로파일 반환."""
        return _PROFILES.get(location)

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def _on_pick_point(self, msg: PickPoint) -> None:
        with self._pick_lock:
            self._latest_pick = msg

    def _goal_cb(self, goal_request) -> GoalResponse:
        loc = goal_request.location
        self.get_logger().info(f"Goal 수락: location={loc}")
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _) -> CancelResponse:
        self.get_logger().info("취소 요청 수락")
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle) -> VisionPick.Result:
        location = goal_handle.request.location
        fb  = VisionPick.Feedback()
        res = VisionPick.Result()

        self.get_logger().info(f"[VisionPick] 실행 시작  location={location}")

        # Phase 1: moving
        self._fb(goal_handle, fb, "moving", 0.10)

        # location 프로파일 로드
        profile = self._load_profile(location)
        if profile is None:
            return self._abort(goal_handle, res, f"알 수 없는 location: '{location}'")

        # fixed_coords 프로파일 (비전 없음) 처리
        if profile.get("observe_pose") is None:
            fixed = profile.get("fixed_coords")
            if not fixed:
                return self._abort(goal_handle, res, f"observe_pose/fixed_coords 미설정: {location}")
            self._fb(goal_handle, fb, "picking", 0.50)
            if not self._do_pick_fixed(fixed, profile):
                return self._abort(goal_handle, res, "fixed pick 동작 실패")
            self._fb(goal_handle, fb, "done", 1.00)
            res.success = True
            res.message = f"location={location} pick 완료 (fixed)"
            goal_handle.succeed()
            return res

        # 비전 기반 pick
        self._send_cv_config(profile)

        # coord_transform 비활성화 (이동 중 잘못된 좌표 발행 방지)
        self._set_coord_transform(False)

        if not self._move_to_observe(location, profile):
            self._set_coord_transform(False)
            return self._abort(goal_handle, res, f"observe_pose 이동 실패: {location}")

        # 이동 완료 → 실제 EE 좌표 읽어서 coord_transform T_base2cam 업데이트
        if self._mc is not None:
            actual = self._mc.get_coords()
            if actual and len(actual) == 6:
                self._update_coord_transform_pose(actual)
            else:
                self.get_logger().warn("get_coords() 실패 — yaml 고정값 사용")

        # coord_transform 활성화
        self._set_coord_transform(True)

        # Phase 2: detecting
        self._fb(goal_handle, fb, "detecting", 0.30)
        pick_pt = self._wait_for_point()
        if pick_pt is None:
            self._set_coord_transform(False)
            return self._abort(goal_handle, res, "검출 타임아웃")

        # 검출 완료 → coord_transform 비활성화
        self._set_coord_transform(False)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success, res.message = False, "Goal 취소됨"
            return res

        # Phase 3: transforming
        self._fb(goal_handle, fb, "transforming", 0.50)
        x_mm    = pick_pt.x * 1000.0
        y_mm    = pick_pt.y * 1000.0
        z_mm    = pick_pt.z * 1000.0
        yaw_deg = pick_pt.yaw_deg
        self.get_logger().info(
            f"[VisionPick] 픽업 좌표: x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm  yaw={yaw_deg:.1f} deg"
        )

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
        self.get_logger().info("[VisionPick] 완료")
        return res

    @staticmethod
    def _fb(gh, fb: VisionPick.Feedback, phase: str, progress: float) -> None:
        fb.phase, fb.progress = phase, progress
        gh.publish_feedback(fb)

    @staticmethod
    def _abort(gh, res: VisionPick.Result, msg: str) -> VisionPick.Result:
        res.success, res.message = False, msg
        gh.abort()
        return res

    def _move_to_observe(self, location: str, profile: Optional[dict]) -> bool:
        # 프로파일에 observe_pose가 있으면 사용, 없으면 기본값
        if profile and "observe_pose" in profile:
            pose = list(profile["observe_pose"])
        else:
            self.get_logger().warn(f"observe_pose 프로파일 없음: {location}")
            return False
        if self._mc is None:
            self.get_logger().info(f"[모의] observe_pose 이동: {pose}")
            time.sleep(1.0)
            return True
        self._mc.set_gripper_value(_GRIPPER_OPEN, 50)
        time.sleep(0.5)
        self._mc.send_coords(pose, _PICK_SPEED, 0)
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

    def _do_pick(self, x_mm: float, y_mm: float, z_mm: float, yaw_deg: float,
                profile: Optional[dict] = None) -> bool:
        # 프로파일에 grasp 자세가 있으면 사용, 없으면 yaml 전역값 사용
        roll       = profile.get("grasp_roll",       self._grasp_roll)    if profile else self._grasp_roll
        pitch      = profile.get("grasp_pitch",      self._grasp_pitch)   if profile else self._grasp_pitch
        yaw_offset = profile.get("grasp_yaw_offset", self._grasp_yaw_off) if profile else self._grasp_yaw_off
        z_offset   = profile.get("pick_z_offset_mm", self._pick_z_off_mm) if profile else self._pick_z_off_mm
        rz = yaw_deg + yaw_offset
        while rz > 180.0:  rz -= 360.0
        while rz < -180.0: rz += 360.0
        rx, ry = roll, pitch
        ox, oy, oz = self._tcp_offset[0], self._tcp_offset[1], self._tcp_offset[2]
        approach = [x_mm + ox, y_mm + oy, z_mm + oz + z_offset, rx, ry, rz]
        pick_pos = [x_mm + ox, y_mm + oy, z_mm + oz, rx, ry, rz]
        if self._mc is None:
            self.get_logger().info(f"[모의] 접근: {approach}  파지: {pick_pos}")
            time.sleep(1.0)
            return True
        self._mc.send_coords(approach, _PICK_SPEED, 0)
        self._wait_move(settle=1.0)
        self._mc.send_coords(pick_pos, _PICK_SPEED // 2, 0)
        self._wait_move(settle=0.5)
        self._mc.set_gripper_value(_GRIPPER_CLOSE, 50)
        time.sleep(1.0)
        self._mc.send_coords(approach, _PICK_SPEED, 0)
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

    def _do_pick_fixed(self, fixed_coords: list, profile: Optional[dict] = None) -> bool:
        """비전 없이 티칭 좌표로 직접 pick."""
        if self._mc is None:
            self.get_logger().info(f"[모의] fixed pick: {fixed_coords}")
            time.sleep(1.0)
            return True
        z_offset   = profile.get("pick_z_offset_mm", self._pick_z_off_mm) if profile else self._pick_z_off_mm
        x, y, z    = fixed_coords[0], fixed_coords[1], fixed_coords[2]
        rx, ry, rz = fixed_coords[3], fixed_coords[4], fixed_coords[5]
        approach   = [x, y, z + z_offset, rx, ry, rz]
        pick_pos   = [x, y, z, rx, ry, rz]
        self._mc.set_gripper_value(_GRIPPER_OPEN, 50)
        time.sleep(0.5)
        self._mc.send_coords(approach, _PICK_SPEED, 0)
        self._wait_move(settle=1.0)
        self._mc.send_coords(pick_pos, _PICK_SPEED // 2, 0)
        self._wait_move(settle=0.5)
        self._mc.set_gripper_value(_GRIPPER_CLOSE, 50)
        time.sleep(1.0)
        self._mc.send_coords(approach, _PICK_SPEED, 0)
        self._wait_move(settle=1.0)
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionPickNode()
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
