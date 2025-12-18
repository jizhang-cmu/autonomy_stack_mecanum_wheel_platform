//
// Created by shibo zhao on 2020-09-27.
//

#ifndef arise_slam_mid360_FEATUREEXTRACTION_H
#define arise_slam_mid360_FEATUREEXTRACTION_H

#include "arise_slam_mid360/logging.h"
#include "arise_slam_mid360/FeatureExtraction/LidarKeypointExtractor.h"
#include "arise_slam_mid360/FeatureExtraction/DepthImageKeypointExtractor.h"


#include <cmath>
#include <string>
#include <vector>

#include <sophus/so3.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <arise_slam_mid360_msgs/msg/laser_feature.hpp>

#include "arise_slam_mid360/common.h"
#include "arise_slam_mid360/container/MapRingBuffer.h"
#include "arise_slam_mid360/sensor_data/imu/imu_data.h"
#include "arise_slam_mid360/sensor_data/pointcloud/point_os.h"
#include "arise_slam_mid360/tic_toc.h"
#include "arise_slam_mid360/utils/Twist.h"
#include "arise_slam_mid360/config/parameter.h"

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include <mutex>
#include <iomanip>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <omp.h>

namespace arise_slam {

    using std::atan2;
    using std::cos;
    using std::sin;
    std::vector<std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>> all_cloud_buf(2);
     
    constexpr unsigned int BLOCK_TIME_NS = 55296;   // Time in ns for one block (measurement + recharge)
    constexpr std::size_t NUM_BLOCKS = 12;    // Number of blocks in a Velodyne packet
    constexpr double LIDAR_MESSAGE_TIME = (double)(NUM_BLOCKS * BLOCK_TIME_NS * 151) * 1e-9;

    constexpr double IMU_TIME_LENIENCY = 0.1;
   

   


    struct bounds_t
    {
        double blindFront;
        double blindBack;
        double blindRight;
        double blindLeft;
        double blindDiskLow;
        double blindDiskHigh;
        double blindDiskRadius;
    };

    struct feature_extraction_config{
        bounds_t box_size;
        int skipFrame;
        bool lidar_flip;
        int N_SCANS;
        int provide_point_time;
        int point_filter_num;
        bool use_dynamic_mask;
        bool use_imu_roll_pitch;
        bool use_up_realsense_points;
        bool use_down_realsense_points;
        bool debug_view_enabled;
        float min_range;
        float max_range;
        int skip_realsense_points;
        SensorType sensor;
        double livox_pitch;

        double imu_acc_x_offset;
        double imu_acc_y_offset;
        double imu_acc_z_offset;
        double imu_acc_x_limit;
        double imu_acc_y_limit;
        double imu_acc_z_limit;

    };

    typedef feature_extraction_config feature_extraction_config;

    class featureExtraction : public rclcpp::Node {
    public:

        /* TODO: return this as a parameter */

        static constexpr double scanPeriod = 0.100859904 - 20.736e-6;
        static constexpr double columnTime = 55.296e-6;
        static constexpr double laserTime = 2.304e-6;

        // for livox
        pcl::PointCloud<pcl::PointXYZINormal> pl_full, pl_corn, pl_surf;
        pcl::PointCloud<pcl::PointXYZINormal> pl_buff[128]; //maximum 128 line lidar
        int scan_count = 0;
        bool is_first_lidar = true;
        double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
        std::deque<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> lidar_buffer;
        bool time_sync_en = false;
        bool timediff_set_flg = false;
        double timediff_lidar_wrt_imu = 0.0;

        featureExtraction(const rclcpp::NodeOptions & options);

        void initInterface();

        template <typename PointT>
        void removeClosestFarestPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                            pcl::PointCloud<PointT> &cloud_out, float min_range, float max_range);
        template <typename Meas>
        bool synchronize_measurements(MapRingBuffer<Meas> &measureBuf,
                                        MapRingBuffer<pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr> &lidarBuf);
        void imuRemovePointDistortion(double lidar_start_time, double lidar_end_time, MapRingBuffer<Imu::Ptr> &imuBuf,
                                    pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg);
        void feature_extraction(double lidar_start_time, pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn);

        void launch_pcl_viewer();

        void convert_pointcloud_format(pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn, pcl::PointCloud<LidarKeypointExtractor::Point>::Ptr &cloud_out);

        void vioRemovePointDistortion(double lidar_start_time, double lidar_end_time, MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr>&vioBuf,
                                    pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg);
        void undistortionAndscanregistration();

        bool synchronizeLidarDepthMeasurement(double lidar_start_time,std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_in, DepthType depthtype);

        void normalizeImuData(const sensor_msgs::msg::Imu::SharedPtr imu_in, Imu::Ptr imuData);

        void imu_Handler(const sensor_msgs::msg::Imu::SharedPtr msg_in);

        void visual_odom_Handler(const nav_msgs::msg::Odometry::SharedPtr visualOdometry);

        void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);

        void livoxHandler(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg);

        void uniformFeatureExtraction(int pl_size, const livox_ros_driver2::msg::CustomMsg::UniquePtr msg);

        void assignTimeforPointCloud(pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_);
        
        template <typename Point>
        sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, typename pcl::PointCloud<Point>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame);
    

        void convert_velodyne_scan_order(pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn);

        bool readParameters();

        void depthCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud0_msg, const sensor_msgs::msg::PointCloud2::SharedPtr cloud1_msg);

        void depthupHandler(const sensor_msgs::msg::PointCloud2::SharedPtr depthupMsg); 

        void depthdownHandler(const sensor_msgs::msg::PointCloud2::SharedPtr depthdownMsg); 
        
        void publishTopic(double lidar_start_time, 
                                         pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr laser_no_distortion_points,
                                         pcl::PointCloud<PointType>::Ptr edgePoints,
                                         pcl::PointCloud<PointType>::Ptr plannerPoints, 
                                         pcl::PointCloud<PointType>::Ptr depthPoints,
                                         Eigen::Quaterniond q_w_original_l);
        
        Eigen::Matrix3d getPitchTF(double pitch_degrees);
        
        Imu::Ptr imu_Init = std::make_shared<Imu>();
        MapRingBuffer<Imu::Ptr> imuBuf;
        MapRingBuffer<pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr> lidarBuf;
        MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> visualOdomBuf;

        //parameters can be changed by config
        std::shared_ptr<LidarKeypointExtractor> KeyPointsExtractor =
                std::make_shared<LidarKeypointExtractor>();

        DepthKeypointExtractor DepthFeatureExtraction;

        typedef sensor_msgs::msg::PointCloud2 DepthMsgType;
        typedef message_filters::sync_policies::ApproximateTime<DepthMsgType, DepthMsgType> syncPolicy;
        message_filters::Synchronizer<syncPolicy> sync{syncPolicy(10)};
        // message_filters::Subscriber<DepthMsgType> sub_depth_up;
        // message_filters::Subscriber<DepthMsgType> sub_depth_down;

    private:
        // ROS Interface
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_depth_up;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_depth_down;

        rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subLivoxCloud;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubEdgePoints;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPlannerPoints;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubBobPoints;
        rclcpp::Publisher<arise_slam_mid360_msgs::msg::LaserFeature>::SharedPtr pubLaserFeatureInfo;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDepthUpPoints;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDepthDownPoints;
        std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubEachScan;

        rclcpp::CallbackGroup::SharedPtr cb_group_;

        int delay_count_;
        std::mutex m_buf;
        int frameCount = 0;

        bool PUB_EACH_LINE = false;
        bool LASER_IMU_SYNC_SCCUESS = false;
        bool LASER_CAMERA_SYNC_SUCCESS = false;
        bool LASER_UP_DEPTH_SYNC_SCCUESS = false;
        bool LASER_DOWN_DEPTH_SYNC_SCCUESS = false;
        bool FIRST_LASER_FRAME=false;
        bool IMU_INIT=false;
        double m_imuPeriod;

        arise_slam_mid360_msgs::msg::LaserFeature laserFeature;
        std_msgs::msg::Header FeatureHeader;
        Eigen::Quaterniond q_w_original_l;
        Eigen::Vector3d t_w_original_l;
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloudwithTime=nullptr;
        pcl::PointCloud<point_os::OusterPointXYZIRT>::Ptr tmpOusterCloudIn=nullptr ;
        pcl::PointCloud<PointType>::Ptr laserCloud_ringorder=nullptr;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudWithFeatures_ringorder=nullptr;
        feature_extraction_config config_;
        DepthType depthtype;

    };


} // namespace arise_slam




#endif //arise_slam_mid360_FEATUREEXTRACTION_H
