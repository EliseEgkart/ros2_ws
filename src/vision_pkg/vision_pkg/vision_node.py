# vision_node.py
import rclpy
from rclpy.node import Node
from arm_interfaces.srv import GetTargetPose
from vision_pkg.vision_6d_manager import (
    COMP_MODEL_PATH,
    DET_MODEL_PATH,
    SEG_MODEL_PATH,
    Vision6DPoseManager,
)

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
        self.get_logger().info('[VISION] loading 6D ensemble vision manager')

        self.declare_parameter('det_model_path', DET_MODEL_PATH)
        self.declare_parameter('seg_model_path', SEG_MODEL_PATH)
        self.declare_parameter('comp_model_path', COMP_MODEL_PATH)
        self.declare_parameter('visualize', False)
        self.declare_parameter('visualize_window', '6D Pose (Ensemble Mode)')
        self.declare_parameter('visualize_scale', 1.0)

        # Depth/RANSAC volume filter parameters
        self.declare_parameter('use_depth_volume_filter', True)
        self.declare_parameter('visualize_depth_mask', False)
        self.declare_parameter('depth_min_height_m', 0.015)
        self.declare_parameter('depth_max_height_m', 0.120)
        self.declare_parameter('depth_min_size_x_m', 0.025)
        self.declare_parameter('depth_min_size_y_m', 0.025)
        self.declare_parameter('depth_min_area_px', 80)
        self.declare_parameter('depth_min_overlap_ratio', 0.06)
        self.declare_parameter('depth_border_margin_px', 8)
        self.declare_parameter('depth_max_border_contact_m', 0.008)
        self.declare_parameter('use_depth_mask_as_yolo_input', False)

        # YOLO segmentation shape-ratio filter parameters
        self.declare_parameter('use_shape_ratio_filter', True)
        self.declare_parameter('shape_ratio_2x2_max', 1.45)
        self.declare_parameter('shape_ratio_4x2_min', 1.55)
        self.declare_parameter('shape_ratio_min_mask_points', 8)

        self.declare_parameter('live_view', False)
        self.declare_parameter('live_target_id', 0)
        self.declare_parameter('live_view_period_sec', 0.15)

        self.vision = None
        self.init_error = None
        self.live_view_timer = None
        try:
            self.vision = Vision6DPoseManager(
                logger=self.get_logger(),
                det_model_path=self.get_parameter('det_model_path').value,
                seg_model_path=self.get_parameter('seg_model_path').value,
                comp_model_path=self.get_parameter('comp_model_path').value,
                visualize=(
                    bool(self.get_parameter('visualize').value) or
                    bool(self.get_parameter('live_view').value)
                ),
                visualize_window=self.get_parameter('visualize_window').value,
                visualize_scale=float(self.get_parameter('visualize_scale').value),

                use_depth_volume_filter=bool(
                    self.get_parameter('use_depth_volume_filter').value
                ),
                visualize_depth_mask=bool(
                    self.get_parameter('visualize_depth_mask').value
                ),
                depth_min_height_m=float(
                    self.get_parameter('depth_min_height_m').value
                ),
                depth_max_height_m=float(
                    self.get_parameter('depth_max_height_m').value
                ),
                depth_min_size_x_m=float(
                    self.get_parameter('depth_min_size_x_m').value
                ),
                depth_min_size_y_m=float(
                    self.get_parameter('depth_min_size_y_m').value
                ),
                depth_min_area_px=int(
                    self.get_parameter('depth_min_area_px').value
                ),
                depth_min_overlap_ratio=float(
                    self.get_parameter('depth_min_overlap_ratio').value
                ),
                depth_border_margin_px=int(
                    self.get_parameter('depth_border_margin_px').value
                ),
                depth_max_border_contact_m=float(
                    self.get_parameter('depth_max_border_contact_m').value
                ),
                use_depth_mask_as_yolo_input=bool(
                    self.get_parameter('use_depth_mask_as_yolo_input').value
                ),

                use_shape_ratio_filter=bool(
                    self.get_parameter('use_shape_ratio_filter').value
                ),
                shape_ratio_2x2_max=float(
                    self.get_parameter('shape_ratio_2x2_max').value
                ),
                shape_ratio_4x2_min=float(
                    self.get_parameter('shape_ratio_4x2_min').value
                ),
                shape_ratio_min_mask_points=int(
                    self.get_parameter('shape_ratio_min_mask_points').value
                ),
            )
            self.get_logger().info('[VISION] vision_node started (6D ensemble based)')
            if bool(self.get_parameter('live_view').value):
                period_sec = max(
                    0.03,
                    float(self.get_parameter('live_view_period_sec').value),
                )
                self.live_view_timer = self.create_timer(period_sec, self.live_view_cb)
                self.get_logger().info(
                    f'[VISION] live view enabled '
                    f'(target_id={self.get_parameter("live_target_id").value}, '
                    f'period={period_sec:.2f}s)'
                )
        except Exception as e:
            self.init_error = str(e)
            self.get_logger().error(f'[VISION] 6D ensemble init failed: {e}')

    def live_view_cb(self):
        if self.vision is None:
            return

        try:
            self.vision.show_live_frame(
                target_id=int(self.get_parameter('live_target_id').value)
            )
        except Exception as e:
            self.get_logger().warn(f'[VISION] live view frame skipped: {e}')

    def get_pose_cb(self, request, response):
        target_str = request.target_color.strip()
        self.get_logger().info(f'[VISION] 서비스 요청 수신 - target ID: {target_str}')

        try:
            if self.vision is None:
                response.success = False
                self.get_logger().error(
                    f'[VISION] unavailable: 6D ensemble init failed: {self.init_error}'
                )
                return response

            if not target_str.isdigit():
                self.get_logger().error(
                    f'[VISION] 잘못된 입력입니다. 숫자 ID를 입력하세요: {target_str}'
                )
                response.success = False
                return response

            target_id = int(target_str)

            result = self.vision.run_pipeline_by_id(target_id=target_id)

            if result.success:
                response.success = True

                # 6D manager already returns ROS-facing units: m, deg.
                response.x = float(result.x_m)
                response.y = float(result.y_m)
                response.z = float(result.z_m)
                response.yaw = float(result.yaw_deg)
                response.class_name = str(result.class_name)

                self.get_logger().info(
                    f'[VISION] 타겟 발견! '
                    f'ID={result.target_id}, '
                    f'Class={result.class_name}, '
                    f'X={result.x_m * 1000.0:.1f}mm, '
                    f'Y={result.y_m * 1000.0:.1f}mm, '
                    f'Z={result.z_m * 1000.0:.1f}mm, '
                    f'Yaw={result.yaw_deg:.2f}deg'
                )

            else:
                response.success = False
                self.get_logger().error(
                    f'[VISION] 타겟 탐색 실패: '
                    f'ID={result.target_id}, '
                    f'Class={result.class_name}, '
                    f'Reason={result.reason}'
                )

        except Exception as e:
            self.get_logger().error(f'[VISION] 처리 중 심각한 오류 발생: {e}')
            response.success = False

        return response

    def destroy_node(self):
        if self.vision is not None:
            self.vision.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()