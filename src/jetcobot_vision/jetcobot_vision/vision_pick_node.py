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

import time
import threading
from typing import Optional

import requests
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_srvs.srv import SetBool

from jetcobot_vision_msgs.action import VisionPick

try:
    from pymycobot import MyCobot280 as _MC280
    _MC_OK = True
except ImportError:
    _MC_OK = False

_PICK_SPEED    = 30
_GRIPPER_OPEN  = 100
_GRIPPER_CLOSE = 0
_MAX_MOVE_WAIT = 30.0


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
        self.declare_parameter("cv_server_url",        "http://localhost:8081")
        self.declare_parameter("coord_enable_service", "/coord_transform_node/enable")

        port                      = self.get_parameter("port").value
        baud                      = self.get_parameter("baud").value
        self._grasp_roll          = self.get_parameter("grasp_roll").value
        self._grasp_pitch         = self.get_parameter("grasp_pitch").value
        self._grasp_yaw_off       = self.get_parameter("grasp_yaw_offset").value
        self._pick_z_off_mm       = self.get_parameter("pick_z_offset_mm").value
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
            except Exception as exc:
                self._mc = None
                self.get_logger().warn(f"MyCobot 연결 실패 — 모의 동작 모드: {exc}")
        else:
            self._mc = None
            self.get_logger().warn("pymycobot 미설치 — 모의 동작 모드")

        self._latest_point: Optional[PointStamped] = None
        self._point_lock = threading.Lock()

        self.create_subscription(
            PointStamped, coord_topic, self._on_pick_point, 10,
            callback_group=self._cb_group,
        )

        # coord_transform_node enable 서비스 클라이언트
        self._coord_enable_client = self.create_client(
            SetBool, coord_enable_service, callback_group=self._cb_group
        )

        self._action_server = ActionServer(
            self, VisionPick, "vision_pick",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info("VisionPickNode Action Server 대기 중")

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
        if not cfg:
            return
        try:
            url = f"{self._cv_server_url}/config"
            requests.post(url, json=cfg, timeout=1.0)
            self.get_logger().info(f"cv_detect_server 파라미터 전송: {cfg}")
        except Exception as exc:
            self.get_logger().warn(f"cv_detect_server /config 전송 실패: {exc}")

    # ── 헬퍼: location 프로파일 로드 ─────────────────────────────────────────

    def _load_profile(self, location: str) -> Optional[dict]:
        """vision_params.yaml의 profiles.{location} 섹션을 파라미터에서 로드."""
        keys = [
            ("observe_pose", None),
            ("z_surface_mm", None),
            ("hsv_lower",    None),
            ("hsv_upper",    None),
            ("min_w",        None),
            ("max_w",        None),
            ("min_h",        None),
            ("max_h",        None),
        ]
        profile = {}
        for key, _ in keys:
            param_name = f"profiles.{location}.{key}"
            try:
                self.declare_parameter(param_name, rclpy.Parameter.Type.NOT_SET)
            except Exception:
                pass
            p = self.get_parameter(param_name)
            if p.type_ != rclpy.Parameter.Type.NOT_SET:
                profile[key] = p.value
        return profile if profile else None

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def _on_pick_point(self, msg: PointStamped) -> None:
        with self._point_lock:
            self._latest_point = msg

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

        # location 프로파일 로드 → cv_detect_server 파라미터 전송
        profile = self._load_profile(location)
        if profile:
            self._send_cv_config(profile)
        else:
            self.get_logger().warn(f"프로파일 없음: '{location}' — 기본 파라미터 사용")

        # coord_transform 비활성화 (이동 중 잘못된 좌표 발행 방지)
        self._set_coord_transform(False)

        if not self._move_to_observe(location, profile):
            self._set_coord_transform(False)
            return self._abort(goal_handle, res, f"observe_pose 이동 실패: {location}")

        # 이동 완료 → coord_transform 활성화
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
        x_mm = pick_pt.point.x * 1000.0
        y_mm = pick_pt.point.y * 1000.0
        z_mm = pick_pt.point.z * 1000.0
        self.get_logger().info(
            f"[VisionPick] 픽업 좌표: x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm"
        )

        # Phase 4: picking
        self._fb(goal_handle, fb, "picking", 0.70)
        if not self._do_pick(x_mm, y_mm, z_mm):
            return self._abort(goal_handle, res, "픽업 동작 실패")

        # Phase 5: done
        self._fb(goal_handle, fb, "done", 1.00)
        res.success = True
        res.message = f"location={location} 픽업 완료"
        res.pick_point_base.x = pick_pt.point.x
        res.pick_point_base.y = pick_pt.point.y
        res.pick_point_base.z = pick_pt.point.z
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

    def _wait_for_point(self) -> Optional[PointStamped]:
        with self._point_lock:
            self._latest_point = None
        deadline = time.monotonic() + self._detect_timeout
        while time.monotonic() < deadline:
            with self._point_lock:
                if self._latest_point is not None:
                    return self._latest_point
            time.sleep(0.05)
        self.get_logger().warn(f"검출 타임아웃 ({self._detect_timeout:.1f}s)")
        return None

    def _do_pick(self, x_mm: float, y_mm: float, z_mm: float) -> bool:
        rx, ry, rz = self._grasp_roll, self._grasp_pitch, self._grasp_yaw_off
        approach = [x_mm, y_mm, z_mm + self._pick_z_off_mm, rx, ry, rz]
        pick_pos = [x_mm, y_mm, z_mm, rx, ry, rz]
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
