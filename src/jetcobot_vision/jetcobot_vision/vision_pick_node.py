#!/usr/bin/env python3
"""
VisionPickNode — Action Server
================================
클라이언트가 VisionPick Goal(location) 을 전송하면:

  Phase 1 "moving"      → observe_pose 로 로봇 이동
  Phase 2 "detecting"   → coord_transform/pick_point_base 토픽 대기
  Phase 3 "transforming"→ 좌표 검증 및 로깅
  Phase 4 "picking"     → 접근 → 하강 → 파지 → 리프트
  Phase 5 "done"        → Result 반환

Action
  /vision_pick  [jetcobot_vision_msgs/action/VisionPick]

파라미터
  port               str    "/dev/ttyJETCOBOT"
  baud               int    1000000
  grasp_roll         float  -178.81  (deg)
  grasp_pitch        float  0.94     (deg)
  grasp_yaw_offset   float  0.0      (deg)
  tcp_offset         float[3]  [0.0, 20.0, 100.0] (mm)
  pick_z_offset_mm   float  20.0   (접근 높이)
  detect_timeout_sec float  10.0
  coord_topic        str    "/coord_transform/pick_point_base"
"""

import time
import threading
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from jetcobot_vision_msgs.action import VisionPick

try:
    from pymycobot import MyCobot280 as _MC280
    _MC_OK = True
except ImportError:
    _MC_OK = False

_PICK_SPEED       = 30    # mm/s (send_coords speed 파라미터)
_GRIPPER_OPEN     = 100
_GRIPPER_CLOSE    = 0
_MAX_MOVE_WAIT    = 30.0  # 이동 완료 대기 최대 초

# workspace_config.yaml 에서 가져온 observe_pose 사전
# 런타임에 파라미터로 덮어쓸 수 없으므로 기본값은 여기에 하드코딩 후
# 파라미터 서버로 전달한다.
_OBSERVE_POSES: dict[str, list[float]] = {
    "tray": [-62.4, -87.9, 314.2, -161.81, 0.94, -179.37],
}


class VisionPickNode(Node):

    def __init__(self) -> None:
        super().__init__("vision_pick_node")

        # ReentrantCallbackGroup: Action 실행 중에도 토픽 콜백이 작동하도록
        self._cb_group = ReentrantCallbackGroup()

        # ── 파라미터 ──────────────────────────────────────────────────────────
        self.declare_parameter("port",               "/dev/ttyJETCOBOT")
        self.declare_parameter("baud",               1_000_000)
        self.declare_parameter("grasp_roll",         -178.81)
        self.declare_parameter("grasp_pitch",         0.94)
        self.declare_parameter("grasp_yaw_offset",    0.0)
        self.declare_parameter("tcp_offset",          [0.0, 20.0, 100.0])
        self.declare_parameter("pick_z_offset_mm",   20.0)
        self.declare_parameter("detect_timeout_sec", 10.0)
        self.declare_parameter("coord_topic",
                               "/coord_transform/pick_point_base")

        port                  = self.get_parameter("port").value
        baud                  = self.get_parameter("baud").value
        self._grasp_roll       = self.get_parameter("grasp_roll").value
        self._grasp_pitch      = self.get_parameter("grasp_pitch").value
        self._grasp_yaw_off    = self.get_parameter("grasp_yaw_offset").value
        self._pick_z_off_mm    = self.get_parameter("pick_z_offset_mm").value
        self._detect_timeout   = self.get_parameter("detect_timeout_sec").value
        coord_topic            = self.get_parameter("coord_topic").value

        # ── pymycobot 연결 ────────────────────────────────────────────────────
        if _MC_OK:
            self.get_logger().info(f"MyCobot 연결 중: {port} @ {baud}")
            self._mc = _MC280(port, baud)
            time.sleep(0.5)
            self.get_logger().info("MyCobot 연결 완료")
        else:
            self._mc = None
            self.get_logger().warn("pymycobot 미설치 — 모의 동작 모드")

        # ── 변환 좌표 수신 ────────────────────────────────────────────────────
        self._latest_point: Optional[PointStamped] = None
        self._point_lock = threading.Lock()

        self.create_subscription(
            PointStamped,
            coord_topic,
            self._on_pick_point,
            10,
            callback_group=self._cb_group,
        )

        # ── Action Server ─────────────────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            VisionPick,
            "vision_pick",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info("VisionPickNode Action Server 대기 중")

    # ── 좌표 수신 콜백 ────────────────────────────────────────────────────────

    def _on_pick_point(self, msg: PointStamped) -> None:
        with self._point_lock:
            self._latest_point = msg

    # ── Action 콜백 ───────────────────────────────────────────────────────────

    def _goal_cb(self, goal_request) -> GoalResponse:
        loc = goal_request.location
        if loc not in _OBSERVE_POSES:
            self.get_logger().warn(f"지원하지 않는 location: '{loc}'")
            return GoalResponse.REJECT
        self.get_logger().info(f"Goal 수락: location={loc}")
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("취소 요청 수락")
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle) -> VisionPick.Result:
        location = goal_handle.request.location
        fb  = VisionPick.Feedback()
        res = VisionPick.Result()

        self.get_logger().info(f"[VisionPick] 실행 시작  location={location}")

        # ── Phase 1: observe_pose 이동 ────────────────────────────────────────
        self._send_feedback(goal_handle, fb, "moving", 0.10)
        if not self._move_to_observe(location):
            return self._abort(goal_handle, res, f"observe_pose 이동 실패: {location}")

        # ── Phase 2: 검출 좌표 대기 ───────────────────────────────────────────
        self._send_feedback(goal_handle, fb, "detecting", 0.30)
        pick_pt = self._wait_for_point()
        if pick_pt is None:
            return self._abort(goal_handle, res, "검출 타임아웃")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Goal 취소됨"
            return res

        # ── Phase 3: 좌표 검증 ────────────────────────────────────────────────
        self._send_feedback(goal_handle, fb, "transforming", 0.50)
        x_mm = pick_pt.point.x * 1000.0
        y_mm = pick_pt.point.y * 1000.0
        z_mm = pick_pt.point.z * 1000.0
        self.get_logger().info(
            f"[VisionPick] 픽업 좌표 (base_link): "
            f"x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm"
        )

        # ── Phase 4: Pick 실행 ────────────────────────────────────────────────
        self._send_feedback(goal_handle, fb, "picking", 0.70)
        if not self._do_pick(x_mm, y_mm, z_mm):
            return self._abort(goal_handle, res, "픽업 동작 실패")

        # ── Phase 5: 완료 ─────────────────────────────────────────────────────
        self._send_feedback(goal_handle, fb, "done", 1.00)

        res.success = True
        res.message = f"location={location} 픽업 완료"
        res.pick_point_base.x = pick_pt.point.x
        res.pick_point_base.y = pick_pt.point.y
        res.pick_point_base.z = pick_pt.point.z
        goal_handle.succeed()
        self.get_logger().info("[VisionPick] 완료")
        return res

    # ── 내부 헬퍼 ─────────────────────────────────────────────────────────────

    @staticmethod
    def _send_feedback(
        gh, fb: VisionPick.Feedback, phase: str, progress: float
    ) -> None:
        fb.phase    = phase
        fb.progress = progress
        gh.publish_feedback(fb)

    @staticmethod
    def _abort(gh, res: VisionPick.Result, msg: str) -> VisionPick.Result:
        res.success = False
        res.message = msg
        gh.abort()
        return res

    def _move_to_observe(self, location: str) -> bool:
        pose = _OBSERVE_POSES.get(location)
        if pose is None:
            self.get_logger().error(f"observe_pose 없음: {location}")
            return False

        if self._mc is None:
            self.get_logger().info(f"[모의] observe_pose 이동: {pose}")
            time.sleep(1.0)
            return True

        self._mc.set_gripper_value(_GRIPPER_OPEN, 50)
        time.sleep(0.5)
        self._mc.send_coords(pose, _PICK_SPEED, 0)
        self._wait_move()
        return True

    def _wait_for_point(self) -> Optional[PointStamped]:
        """coord_transform 토픽에서 최신 좌표를 detect_timeout_sec 내 수신."""
        with self._point_lock:
            self._latest_point = None  # 이전 캐시 초기화

        deadline = time.monotonic() + self._detect_timeout
        while time.monotonic() < deadline:
            with self._point_lock:
                if self._latest_point is not None:
                    return self._latest_point
            time.sleep(0.05)

        self.get_logger().warn(
            f"검출 타임아웃 ({self._detect_timeout:.1f}s 초과)"
        )
        return None

    def _do_pick(self, x_mm: float, y_mm: float, z_mm: float) -> bool:
        """
        비전 좌표 기반 Pick 시퀀스.
        workspace_config.yaml 의 grasp_rp 파라미터 적용.
        OBB theta 기반 yaw 보정은 grasp_yaw_offset 에 추가 예정.
        """
        rx = self._grasp_roll
        ry = self._grasp_pitch
        rz = self._grasp_yaw_off

        approach = [x_mm, y_mm, z_mm + self._pick_z_off_mm, rx, ry, rz]
        pick_pos  = [x_mm, y_mm, z_mm,                       rx, ry, rz]

        if self._mc is None:
            self.get_logger().info(f"[모의] 접근: {approach}")
            time.sleep(0.5)
            self.get_logger().info(f"[모의] 하강 & 파지: {pick_pos}")
            time.sleep(0.5)
            return True

        # 접근
        self._mc.send_coords(approach, _PICK_SPEED, 0)
        self._wait_move()
        time.sleep(0.2)

        # 하강
        self._mc.send_coords(pick_pos, _PICK_SPEED // 2, 0)
        self._wait_move()
        time.sleep(0.3)

        # 파지
        self._mc.set_gripper_value(_GRIPPER_CLOSE, 50)
        time.sleep(1.0)

        # 리프트
        self._mc.send_coords(approach, _PICK_SPEED, 0)
        self._wait_move()

        return True

    def _wait_move(self) -> None:
        """is_moving() 폴링으로 이동 완료 대기. 타임아웃 시 강제 진행."""
        deadline = time.monotonic() + _MAX_MOVE_WAIT
        while True:
            try:
                moving = bool(self._mc.is_moving()) if self._mc else False
            except Exception as exc:
                self.get_logger().warn(f"is_moving() 예외: {exc} — 강제 진행")
                break
            if not moving:
                break
            if time.monotonic() > deadline:
                self.get_logger().warn("이동 타임아웃 — 강제 진행")
                break
            time.sleep(0.1)


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
