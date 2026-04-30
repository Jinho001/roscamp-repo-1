#!/usr/bin/env python3
"""
VisionPickNode — Action Server
================================
goal.location → observe → detect → transform → pick

Phase 1 "moving"       : observe_pose 로 이동
Phase 2 "detecting"    : pick_point_base 토픽 대기
Phase 3 "transforming" : 좌표 검증 로깅
Phase 4 "picking"      : 접근 → 하강 → 파지 → 리프트
Phase 5 "done"         : Result 반환

Action  /vision_pick  [jetcobot_vision_msgs/action/VisionPick]
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

_PICK_SPEED    = 30
_GRIPPER_OPEN  = 100
_GRIPPER_CLOSE = 0
_MAX_MOVE_WAIT = 30.0

# workspace_config.yaml observe_pose 매핑
_OBSERVE_POSES: dict[str, list[float]] = {
    "tray": [-62.4, -87.9, 314.2, -161.81, 0.94, -179.37],
}


class VisionPickNode(Node):

    def __init__(self) -> None:
        super().__init__("vision_pick_node")

        # ReentrantCallbackGroup: Action 실행 중에도 토픽 콜백 병렬 동작
        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter("port",               "/dev/ttyJETCOBOT")
        self.declare_parameter("baud",               1_000_000)
        self.declare_parameter("grasp_roll",         -178.81)
        self.declare_parameter("grasp_pitch",          0.94)
        self.declare_parameter("grasp_yaw_offset",     0.0)
        self.declare_parameter("tcp_offset",          [0.0, 20.0, 100.0])
        self.declare_parameter("pick_z_offset_mm",    20.0)
        self.declare_parameter("detect_timeout_sec",  10.0)
        self.declare_parameter("coord_topic",  "/coord_transform/pick_point_base")

        port                = self.get_parameter("port").value
        baud                = self.get_parameter("baud").value
        self._grasp_roll    = self.get_parameter("grasp_roll").value
        self._grasp_pitch   = self.get_parameter("grasp_pitch").value
        self._grasp_yaw_off = self.get_parameter("grasp_yaw_offset").value
        self._pick_z_off_mm = self.get_parameter("pick_z_offset_mm").value
        self._detect_timeout = self.get_parameter("detect_timeout_sec").value
        coord_topic         = self.get_parameter("coord_topic").value

        if _MC_OK:
            self.get_logger().info(f"MyCobot 연결 중: {port} @ {baud}")
            self._mc = _MC280(port, baud)
            time.sleep(0.5)
            self.get_logger().info("MyCobot 연결 완료")
        else:
            self._mc = None
            self.get_logger().warn("pymycobot 미설치 — 모의 동작 모드")

        self._latest_point: Optional[PointStamped] = None
        self._point_lock = threading.Lock()

        self.create_subscription(
            PointStamped, coord_topic, self._on_pick_point, 10,
            callback_group=self._cb_group,
        )

        self._action_server = ActionServer(
            self, VisionPick, "vision_pick",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info("VisionPickNode Action Server 대기 중")

    def _on_pick_point(self, msg: PointStamped) -> None:
        with self._point_lock:
            self._latest_point = msg

    def _goal_cb(self, goal_request) -> GoalResponse:
        loc = goal_request.location
        if loc not in _OBSERVE_POSES:
            self.get_logger().warn(f"지원하지 않는 location: '{loc}'")
            return GoalResponse.REJECT
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

        self._fb(goal_handle, fb, "moving", 0.10)
        if not self._move_to_observe(location):
            return self._abort(goal_handle, res, f"observe_pose 이동 실패: {location}")

        self._fb(goal_handle, fb, "detecting", 0.30)
        pick_pt = self._wait_for_point()
        if pick_pt is None:
            return self._abort(goal_handle, res, "검출 타임아웃")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success, res.message = False, "Goal 취소됨"
            return res

        self._fb(goal_handle, fb, "transforming", 0.50)
        x_mm = pick_pt.point.x * 1000.0
        y_mm = pick_pt.point.y * 1000.0
        z_mm = pick_pt.point.z * 1000.0
        self.get_logger().info(
            f"[VisionPick] 픽업 좌표: x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm"
        )

        self._fb(goal_handle, fb, "picking", 0.70)
        if not self._do_pick(x_mm, y_mm, z_mm):
            return self._abort(goal_handle, res, "픽업 동작 실패")

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

    def _move_to_observe(self, location: str) -> bool:
        pose = _OBSERVE_POSES.get(location)
        if pose is None:
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
        """이동 완료 폴링 + 안정화 대기.

        is_moving()이 일부 펌웨어에서 항상 False를 반환하는 알려진 버그가 있어
        최소 settle 초는 무조건 대기한다.
        """
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
