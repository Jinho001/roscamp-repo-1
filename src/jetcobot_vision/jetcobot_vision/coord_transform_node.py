#!/usr/bin/env python3
"""
CoordTransformNode
==================
역할
  1. hand-eye 행렬(T_ee2cam) + observe_pose 로
     base_link → camera_link Static TF 브로드캐스트
  2. ObbBoxArray 구독 → Ray-Plane 교점으로 픽셀 → base_link 3D 변환
     → PointStamped 발행

좌표 변환 수식
  T_base2cam = T_base2ee @ T_ee2cam
  ray_c      = K_inv @ [px, py, 1]
  t          = (z_surface - origin_b.z) / ray_b.z
  point_base = origin_b + t * ray_b

파라미터
  handeye_matrix     float64[16]  행 우선 4×4 (handeye_result.yaml)
  camera_intrinsics  float64[4]   [fx, fy, cx, cy] (camera_info.yaml)
  observe_pose       float64[6]   [x mm, y mm, z mm, rx deg, ry deg, rz deg]
  z_surface_mm       float        작업면 높이 (base_link 기준, mm)
  obb_topic          str          구독할 ObbBoxArray 토픽
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_ros import StaticTransformBroadcaster
import tf2_ros
from std_msgs.msg import Header
from std_srvs.srv import SetBool
from jetcobot_vision_msgs.msg import ObbBoxArray, PickPoint
from jetcobot_vision_msgs.srv import UpdatePose


def _euler_deg_to_rotation(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """MyCobot280 get_coords() rx/ry/rz (deg) → 3×3 rotation matrix (ZYX extrinsic).
    레거시 coord_transform.py의 euler_zyx_to_rotation과 동일한 구현.
    """
    rx, ry, rz = map(math.radians, [rx_deg, ry_deg, rz_deg])
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0],
                   [math.sin(rz),  math.cos(rz), 0],
                   [0,             0,             1]])
    Ry = np.array([[ math.cos(ry), 0, math.sin(ry)],
                   [0,             1, 0            ],
                   [-math.sin(ry), 0, math.cos(ry)]])
    Rx = np.array([[1, 0,            0           ],
                   [0, math.cos(rx), -math.sin(rx)],
                   [0, math.sin(rx),  math.cos(rx)]])
    return Rz @ Ry @ Rx


def _make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t
    return T


def _rot_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """3×3 rotation matrix → (x, y, z, w) quaternion (Shepperd)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s, 0.25/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s


class CoordTransformNode(Node):

    def __init__(self) -> None:
        super().__init__("coord_transform_node")

        # 기본값은 handeye_result.yaml / camera_info.yaml / workspace_config.yaml 에서 가져옴
        _default_he = [
            0.9955181180385095,  -0.09236211587032309, -0.02032033978606575,  0.0004237468556380977,
            0.09215089442325214,  0.9956832022377156,  -0.011098352969840833, -0.07245503146972071,
            0.021257688351718585, 0.009176073975592695,  0.9997319192425216,   0.04859228254585788,
            0.0, 0.0, 0.0, 1.0,
        ]
        _default_K   = [990.8120239698858, 987.6160053978392, 301.19713298628164, 238.38042557352367]
        _default_obs = [-62.4, -87.9, 314.2, -161.81, 0.94, -179.37]

        self.declare_parameter("handeye_matrix",    _default_he)
        self.declare_parameter("camera_intrinsics", _default_K)
        self.declare_parameter("observe_pose",      _default_obs)
        self.declare_parameter("z_surface_mm",      95.0)
        self.declare_parameter("obb_topic",         "/detect_bridge_node/obb_boxes")

        he_flat   = list(self.get_parameter("handeye_matrix").value)
        K_flat    = list(self.get_parameter("camera_intrinsics").value)
        obs_pose  = list(self.get_parameter("observe_pose").value)
        z_surf_mm = float(self.get_parameter("z_surface_mm").value)
        obb_topic = self.get_parameter("obb_topic").value

        self._z_surface_m = z_surf_mm / 1000.0
        self._T_ee2cam    = np.array(he_flat, dtype=np.float64).reshape(4, 4)

        t_ee       = np.array(obs_pose[:3], dtype=np.float64) / 1000.0
        R_ee       = _euler_deg_to_rotation(obs_pose[3], obs_pose[4], obs_pose[5])
        T_base2ee  = _make_T(R_ee, t_ee)
        self._T_base2cam = T_base2ee @ self._T_ee2cam

        fx, fy, cx, cy = K_flat
        self._K_inv = np.linalg.inv(np.array(
            [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64
        ))

        self._static_br = StaticTransformBroadcaster(self)
        self._broadcast_tf(self._T_base2cam)
        self.get_logger().info("Static TF 브로드캐스트: base_link → camera_link")

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pt_pub   = self.create_publisher(PointStamped, "~/pick_point_base", 10)
        self._pick_pub = self.create_publisher(PickPoint, "~/pick_point", 10)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        _cb = ReentrantCallbackGroup()
        self.create_subscription(ObbBoxArray, obb_topic, self._on_obb, sensor_qos, callback_group=_cb)

        # 활성화 플래그 — vision_pick_node가 서비스로 제어
        self._enabled = False
        self.create_service(SetBool, "~/enable", self._on_enable_srv, callback_group=_cb)
        self.create_service(UpdatePose, "~/update_pose", self._on_update_pose, callback_group=_cb)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(f"CoordTransformNode 시작 완료  (z_surface={z_surf_mm} mm)")

    def _broadcast_tf(self, T: np.ndarray) -> None:
        ts = TransformStamped()
        ts.header.stamp    = self.get_clock().now().to_msg()
        ts.header.frame_id = "base_link"
        ts.child_frame_id  = "camera_link"
        t = T[:3, 3]
        ts.transform.translation.x = float(t[0])
        ts.transform.translation.y = float(t[1])
        ts.transform.translation.z = float(t[2])
        qx, qy, qz, qw = _rot_to_quat(T[:3, :3])
        ts.transform.rotation.x = qx
        ts.transform.rotation.y = qy
        ts.transform.rotation.z = qz
        ts.transform.rotation.w = qw
        self._static_br.sendTransform(ts)

    def _on_update_pose(self, req: UpdatePose.Request, res: UpdatePose.Response) -> UpdatePose.Response:
        if len(req.coords) != 6:
            res.success, res.message = False, "coords 길이 6 필요"
            return res
        obs = list(req.coords)
        t_ee      = np.array(obs[:3], dtype=np.float64) / 1000.0
        R_ee      = _euler_deg_to_rotation(obs[3], obs[4], obs[5])
        T_base2ee = _make_T(R_ee, t_ee)
        self._T_base2cam = T_base2ee @ self._T_ee2cam
        self._broadcast_tf(self._T_base2cam)
        self.get_logger().info(f"T_base2cam 업데이트: {[round(c,2) for c in obs]}")
        res.success, res.message = True, "업데이트 완료"
        return res

    def _on_param_change(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "z_surface_mm":
                self._z_surface_m = float(p.value) / 1000.0
                self.get_logger().info(f"z_surface_mm 업데이트: {p.value} mm")
        return SetParametersResult(successful=True)

    def _on_enable_srv(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self._enabled = req.data
        res.success = True
        res.message = f"enabled={self._enabled}"
        self.get_logger().info(f"CoordTransform {'활성화' if self._enabled else '비활성화'}")
        return res

    def _on_obb(self, msg: ObbBoxArray) -> None:
        if not self._enabled or not msg.boxes:
            return

        stamp  = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id="base_link")

        # 최고 confidence 상자 선택 (기존 호환성 유지)
        best_idx = max(range(len(msg.boxes)), key=lambda i: msg.boxes[i].confidence)
        best = msg.boxes[best_idx]

        point_m = self._pixel_to_base(best.cx, best.cy)
        if point_m is None:
            return
        yaw_deg = self._theta_to_yaw(best.theta)

        # 기존 PointStamped 토픽 유지 (최고 confidence 상자만)
        out = PointStamped()
        out.header = header
        out.point.x, out.point.y, out.point.z = float(point_m[0]), float(point_m[1]), float(point_m[2])
        self._pt_pub.publish(out)

        # 모든 상자에 대해 PickPoint 토픽 발행
        for idx, box in enumerate(msg.boxes):
            pt_m = self._pixel_to_base(box.cx, box.cy)
            if pt_m is None:
                continue
            yaw = self._theta_to_yaw(box.theta)

            pp = PickPoint()
            pp.header = header
            pp.x, pp.y, pp.z = float(pt_m[0]), float(pt_m[1]), float(pt_m[2])
            pp.yaw_deg = yaw
            pp.box_index = idx
            pp.confidence = box.confidence
            self._pick_pub.publish(pp)

        self.get_logger().info(
            f"base_link 변환: {len(msg.boxes)}개 상자 발행  "
            f"best[{best_idx}] x={point_m[0]*1000:.1f} mm  "
            f"y={point_m[1]*1000:.1f} mm  z={point_m[2]*1000:.1f} mm  "
            f"yaw={yaw_deg:.1f} deg  (conf={best.confidence:.2f})"
        )

    def _theta_to_yaw(self, theta_cam: float) -> float:
        """카메라 픽셀 좌표계의 OBB 장축 각도(rad) → base_link yaw (deg).
        레거시 theta_cam_to_robot()과 동일한 구현.
        """
        d_cam  = np.array([np.cos(theta_cam), np.sin(theta_cam), 0.0])
        d_base = self._T_base2cam[:3, :3] @ d_cam
        return float(np.degrees(np.arctan2(d_base[1], d_base[0])))

    def _pixel_to_base(self, px: float, py: float) -> np.ndarray | None:
        """픽셀 (px, py) → base_link 3D 좌표 (m). Ray-Plane 교점.
        레거시 coord_transform.py와 동일한 방식: R_base2cam @ ray_cam 직접 사용.
        """
        ray_c  = self._K_inv @ np.array([px, py, 1.0], dtype=np.float64)
        R      = self._T_base2cam[:3, :3]
        t_orig = self._T_base2cam[:3, 3]
        ray_b  = R @ ray_c

        if abs(ray_b[2]) < 1e-9:
            self.get_logger().warn("ray_b.z ≈ 0: 작업면과 평행 — 변환 건너뜀")
            return None
        t_param = (self._z_surface_m - t_orig[2]) / ray_b[2]
        if t_param < 0.0:
            self.get_logger().warn("교점이 카메라 뒤쪽 — 변환 건너뜀")
            return None
        return t_orig + t_param * ray_b


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoordTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
