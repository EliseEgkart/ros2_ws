#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import DetectionArray
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs

class BBoxProjectorNode(Node):
    def __init__(self):
        super().__init__('bbox_projector_node')

        # 1) 파라미터 선언
        self.declare_parameter('detection_topic', '/detections/front_up')
        self.declare_parameter('lidar_topic', '/lidar_preprocessed')
        self.declare_parameter('camera_frame', 'camera_front_up')   # TF 브로드캐스트에서 child_frame_id
        self.declare_parameter('lidar_frame', 'velodyne')          # LiDAR 원점(frame_id)

        self.dt_topic   = self.get_parameter('detection_topic').value
        self.lidar_topic= self.get_parameter('lidar_topic').value
        self.cam_frame  = self.get_parameter('camera_frame').value
        self.lidar_frame= self.get_parameter('lidar_frame').value

        # 2) TF setup
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3) 구독자
        self.sub_det   = self.create_subscription(
            DetectionArray, self.dt_topic, self._on_detection, 10)
        self.sub_lidar = self.create_subscription(
            PointCloud2,    self.lidar_topic, self._on_lidar, 10)

        # 4) 퍼블리셔
        self.pub_markers = self.create_publisher(MarkerArray, '/fused_markers', 10)

        # 최신 메시지 저장
        self._latest_det   = None
        self._latest_lidar = None

        self.get_logger().info('BBoxProjectorNode initialized')

    def _on_detection(self, msg: DetectionArray):
        self._latest_det = msg
        self._try_fuse()

    def _on_lidar(self, msg: PointCloud2):
        self._latest_lidar = msg
        self._try_fuse()

    def _try_fuse(self):
        if self._latest_det is None or self._latest_lidar is None:
            return

        # TF: camera_frame ← lidar_frame at same timestamp
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.cam_frame,
                self._latest_lidar.header.frame_id,
                self._latest_lidar.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        markers = MarkerArray()
        idx = 0

        for det in self._latest_det.detections:
            # 3D bbox center
            ctr = det.bbox3d.center.position
            # bbox3d.frame_id 가 설정되어 있으면 그 프레임, 없으면 camera_frame 사용
            src_frame = det.bbox3d.frame_id or self.cam_frame

            # 메모리 상으로 TransformStamped 를 수정
            tf.header.stamp = self._latest_lidar.header.stamp
            tf.child_frame_id = src_frame

            # geometry_msgs/PointStamped 으로 감싸기
            pt_cam = tf2_geometry_msgs.PointStamped()
            pt_cam.header.frame_id = src_frame
            pt_cam.header.stamp    = self._latest_lidar.header.stamp
            pt_cam.point.x = ctr.x
            pt_cam.point.y = ctr.y
            pt_cam.point.z = ctr.z

            # 카메라 프레임 → LiDAR 프레임 변환
            try:
                pt_lidar = tf2_geometry_msgs.do_transform_point(pt_cam, tf)
            except Exception as e:
                self.get_logger().warn(f'Point transform failed: {e}')
                continue

            # Marker 생성
            m = Marker()
            m.header.frame_id = self.lidar_frame
            m.header.stamp    = self._latest_lidar.header.stamp
            m.ns              = 'fused'
            m.id              = idx
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.pose.position.x = pt_lidar.point.x
            m.pose.position.y = pt_lidar.point.y
            m.pose.position.z = pt_lidar.point.z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            # 클래스별 색상 (예: cone_Y → 노란색)
            if det.class_name.endswith('_Y'):
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.0, 1.0, 1.0

            markers.markers.append(m)
            idx += 1

        self.pub_markers.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = BBoxProjectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()