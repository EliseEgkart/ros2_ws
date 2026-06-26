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

        self.vision = None
        self.init_error = None
        try:
            self.vision = Vision6DPoseManager(
                logger=self.get_logger(),
                det_model_path=self.get_parameter('det_model_path').value,
                seg_model_path=self.get_parameter('seg_model_path').value,
                comp_model_path=self.get_parameter('comp_model_path').value,
                visualize=bool(self.get_parameter('visualize').value),
                visualize_window=self.get_parameter('visualize_window').value,
            )
            self.get_logger().info('[VISION] vision_node started (6D ensemble based)')
        except Exception as e:
            self.init_error = str(e)
            self.get_logger().error(f'[VISION] 6D ensemble init failed: {e}')


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
