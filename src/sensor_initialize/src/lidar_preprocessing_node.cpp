#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

class LidarPreprocessor : public rclcpp::Node
{
public:
    LidarPreprocessor()
    : Node("lidar_preprocessing_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10,
            std::bind(&LidarPreprocessor::cloudCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    }

private:
    // ======= 조정 가능한 파라미터들 =======
    const std::string input_topic_ = "/velodyne_points";
    const std::string output_topic_ = "/lidar_preprocessed";

    // Voxel Grid 필터 설정 (단위: m)
    const float voxel_leaf_size_ = 0.05f;

    // RANSAC 설정
    const double ground_dist_thresh_ = 0.1;

    // ROI 설정 (단위: m)
    const float roi_min_x_ = -0.5f, roi_max_x_ = 5.5f;
    const float roi_min_y_ = -2.5f, roi_max_y_ = 2.5f;
    const float roi_min_z_ = -0.5f, roi_max_z_ = 1.0f;

    // =====================================

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        // [1] ROS → PCL 변환
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*input, *cloud_in);

        // [2] Voxel Grid 필터링 (다운샘플링)
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(cloud_in);
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel.filter(*cloud_voxel);

        // // [3] RANSAC 기반 Ground 제거
        // pcl::SACSegmentation<pcl::PointXYZI> seg;
        // seg.setOptimizeCoefficients(true);
        // seg.setModelType(pcl::SACMODEL_PLANE);
        // seg.setMethodType(pcl::SAC_RANSAC);
        // seg.setDistanceThreshold(ground_dist_thresh_);
        // seg.setInputCloud(cloud_voxel);

        // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // seg.segment(*inliers, *coefficients);

        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZI>());
        // pcl::ExtractIndices<pcl::PointXYZI> extract;
        // extract.setInputCloud(cloud_voxel);
        // extract.setIndices(inliers);
        // extract.setNegative(true);  // inlier 제거 (즉, ground 제거)
        // extract.filter(*cloud_no_ground);

        // [4] ROI 필터링
        pcl::PassThrough<pcl::PointXYZI> pass;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>());

        pass.setInputCloud(cloud_voxel);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(roi_min_x_, roi_max_x_);
        pass.filter(*cloud_roi);

        pass.setInputCloud(cloud_roi);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(roi_min_y_, roi_max_y_);
        pass.filter(*cloud_roi);

        pass.setInputCloud(cloud_roi);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(roi_min_z_, roi_max_z_);
        pass.filter(*cloud_roi);

        // [5] PCL → ROS 메시지 변환 및 Publish
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_roi, output);
        output.header = input->header;
        publisher_->publish(output);
    }

    // ROS 2 통신 객체
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPreprocessor>());
    rclcpp::shutdown();
    return 0;
}