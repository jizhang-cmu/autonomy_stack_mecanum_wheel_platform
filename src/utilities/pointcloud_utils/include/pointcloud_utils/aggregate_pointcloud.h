#ifndef POINTCLOUD_UTILS__AGGREGATE_POINTCLOUD_H_
#define POINTCLOUD_UTILS__AGGREGATE_POINTCLOUD_H_

#include <chrono>
#include <ctime>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

class AggregatePointCloud : public rclcpp::Node
{
public:
    AggregatePointCloud();
    ~AggregatePointCloud();

private:
    // Callback for incoming point clouds
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Service handlers
    void handleClearRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleSaveMapRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Timer callback to publish accumulated cloud
    void publishAccumulatedCloud();

    // Save point cloud to file
    bool savePointCloudToPCD(const std::string& base_filepath);

    // Parameters
    double viz_voxel_size_;
    double save_voxel_size_;
    int max_points_;
    double publish_rate_;
    std::string input_topic_;
    std::string output_topic_;
    std::string save_directory_;
    std::string frame_id_;
    bool save_ply_;

    // Point clouds - separate for visualization (lower res) and saving (higher res)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr viz_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr save_cloud_;

    // Voxel filters
    pcl::VoxelGrid<pcl::PointXYZRGB> viz_voxel_filter_;
    pcl::VoxelGrid<pcl::PointXYZRGB> save_voxel_filter_;

    // Mutex for thread safety
    std::mutex cloud_mutex_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_cloud_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

#endif  // POINTCLOUD_UTILS__AGGREGATE_POINTCLOUD_H_
