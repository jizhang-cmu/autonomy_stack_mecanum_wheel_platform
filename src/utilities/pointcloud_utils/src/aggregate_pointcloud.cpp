#include "pointcloud_utils/aggregate_pointcloud.h"

AggregatePointCloud::AggregatePointCloud()
    : Node("aggregate_pointcloud")
{
    // Declare parameters with defaults
    this->declare_parameter<double>("viz_voxel_size", 0.1);
    this->declare_parameter<double>("save_voxel_size", 0.025);
    this->declare_parameter<int>("max_points", 1000000);
    this->declare_parameter<double>("publish_rate", 1.0);
    this->declare_parameter<std::string>("input_topic", "/registered_scan");
    this->declare_parameter<std::string>("output_topic", "/aggregated_pointcloud");
    this->declare_parameter<std::string>("save_directory", "/tmp");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<bool>("save_ply", true);

    // Get parameters
    viz_voxel_size_ = this->get_parameter("viz_voxel_size").as_double();
    save_voxel_size_ = this->get_parameter("save_voxel_size").as_double();
    max_points_ = this->get_parameter("max_points").as_int();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    save_directory_ = this->get_parameter("save_directory").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    save_ply_ = this->get_parameter("save_ply").as_bool();

    // Initialize both clouds
    viz_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    save_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    viz_cloud_->header.frame_id = frame_id_;
    save_cloud_->header.frame_id = frame_id_;

    // Configure voxel filters
    viz_voxel_filter_.setLeafSize(viz_voxel_size_, viz_voxel_size_, viz_voxel_size_);
    save_voxel_filter_.setLeafSize(save_voxel_size_, save_voxel_size_, save_voxel_size_);

    // Create subscriber
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_,
        10,
        std::bind(&AggregatePointCloud::cloudCallback, this, std::placeholders::_1));

    // Create publisher
    accumulated_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_,
        10);

    // Create services
    clear_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/clear",
        std::bind(&AggregatePointCloud::handleClearRequest, this,
                 std::placeholders::_1, std::placeholders::_2));

    save_map_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/save_map",
        std::bind(&AggregatePointCloud::handleSaveMapRequest, this,
                 std::placeholders::_1, std::placeholders::_2));

    // Create publish timer
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&AggregatePointCloud::publishAccumulatedCloud, this));

    RCLCPP_INFO(this->get_logger(), "Aggregate PointCloud node initialized");
    RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Viz voxel size: %.3f m", viz_voxel_size_);
    RCLCPP_INFO(this->get_logger(), "  Save voxel size: %.3f m", save_voxel_size_);
    RCLCPP_INFO(this->get_logger(), "  Max points: %d", max_points_);
    RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", publish_rate_);
    RCLCPP_INFO(this->get_logger(), "  Save directory: %s", save_directory_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Save PLY: %s", save_ply_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Services:");
    RCLCPP_INFO(this->get_logger(), "  ~/clear - Clear accumulated clouds");
    RCLCPP_INFO(this->get_logger(), "  ~/save_map - Save high-res cloud to PCD%s",
                save_ply_ ? " and PLY" : "");
}

AggregatePointCloud::~AggregatePointCloud()
{
    // Auto-save on exit
    RCLCPP_INFO(this->get_logger(), "Shutting down - auto-saving point cloud...");
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm);

    std::string base_filepath = save_directory_ + "/" + std::string(timestamp) + "_map";
    if (savePointCloudToPCD(base_filepath)) {
        RCLCPP_INFO(this->get_logger(), "Auto-saved point cloud on exit");
    }
}

void AggregatePointCloud::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert ROS message to PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr incoming_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *incoming_cloud);

    if (incoming_cloud->empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(cloud_mutex_);

    // Add incoming points to both clouds
    *viz_cloud_ += *incoming_cloud;
    *save_cloud_ += *incoming_cloud;

    // Apply voxel filter to visualization cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr viz_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    viz_voxel_filter_.setInputCloud(viz_cloud_);
    viz_voxel_filter_.filter(*viz_filtered);
    viz_cloud_ = viz_filtered;

    // Apply voxel filter to save cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr save_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    save_voxel_filter_.setInputCloud(save_cloud_);
    save_voxel_filter_.filter(*save_filtered);
    save_cloud_ = save_filtered;

    // Limit visualization cloud size
    if (viz_cloud_->size() > static_cast<size_t>(max_points_)) {
        viz_cloud_->points.erase(
            viz_cloud_->points.begin(),
            viz_cloud_->points.begin() + (viz_cloud_->size() - max_points_));
        viz_cloud_->width = viz_cloud_->size();
        viz_cloud_->height = 1;

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                            "Viz cloud exceeded max points, oldest points removed");
    }

    RCLCPP_DEBUG(this->get_logger(), "Viz cloud: %zu points, Save cloud: %zu points",
                viz_cloud_->size(), save_cloud_->size());
}

void AggregatePointCloud::handleClearRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    std::lock_guard<std::mutex> lock(cloud_mutex_);

    size_t viz_size = viz_cloud_->size();
    size_t save_size = save_cloud_->size();
    viz_cloud_->clear();
    save_cloud_->clear();

    response->success = true;
    response->message = "Cleared viz cloud (" + std::to_string(viz_size) +
                       " points) and save cloud (" + std::to_string(save_size) + " points)";

    RCLCPP_INFO(this->get_logger(), "Clouds cleared (viz: %zu, save: %zu)", viz_size, save_size);
}

void AggregatePointCloud::handleSaveMapRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    // Generate timestamped filename
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm);

    std::string base_filepath = save_directory_ + "/" + std::string(timestamp) + "_map";

    if (savePointCloudToPCD(base_filepath)) {
        response->success = true;
        response->message = "Point cloud saved successfully to " + base_filepath + ".pcd";
        if (save_ply_) {
            response->message += " and " + base_filepath + ".ply";
        }
    } else {
        response->success = false;
        response->message = "Failed to save point cloud";
        RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud");
    }
}

bool AggregatePointCloud::savePointCloudToPCD(const std::string& base_filepath)
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    if (save_cloud_->empty()) {
        RCLCPP_WARN(this->get_logger(), "Save cloud is empty, nothing to save");
        return false;
    }

    // Create directory if it doesn't exist
    std::filesystem::path file_path(base_filepath + ".pcd");
    std::filesystem::path dir_path = file_path.parent_path();

    try {
        if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
            std::filesystem::create_directories(dir_path);
        }

        // Save as binary compressed PCD
        std::string pcd_filepath = base_filepath + ".pcd";
        if (pcl::io::savePCDFileBinaryCompressed(pcd_filepath, *save_cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "PCD save failed");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Saved %zu points (voxel: %.3fm) to %s",
                   save_cloud_->size(), save_voxel_size_, pcd_filepath.c_str());

        // Optionally save as PLY
        if (save_ply_) {
            std::string ply_filepath = base_filepath + ".ply";
            if (pcl::io::savePLYFile(ply_filepath, *save_cloud_, true) == -1) {
                RCLCPP_WARN(this->get_logger(), "PLY save failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "Also saved to %s", ply_filepath.c_str());
            }
        }

        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while saving: %s", e.what());
        return false;
    }
}

void AggregatePointCloud::publishAccumulatedCloud()
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    if (viz_cloud_->empty()) {
        return;
    }

    // Convert to ROS message and publish (visualization cloud)
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*viz_cloud_, output_msg);
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = frame_id_;

    accumulated_cloud_pub_->publish(output_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published viz cloud: %zu points", viz_cloud_->size());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AggregatePointCloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
