//
// Created by shibo zhao on 2020-09-27.
//

#ifndef arise_slam_mid360_LASERMAPPING_H
#define arise_slam_mid360_LASERMAPPING_H

#include <cmath>
#include <iostream>
#include <queue>
#include <string>
#include <vector>
#include <iomanip>
#include <mutex>
#include <thread>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
// #include <tf_conversions/tf_eigen.h>
#include <std_msgs/msg/float32.hpp>
#include "arise_slam_mid360/LidarProcess/LocalMap.h"
#include "arise_slam_mid360/common.h"
#include "arise_slam_mid360/tic_toc.h"
#include "arise_slam_mid360/utils/Twist.h"
#include "arise_slam_mid360/container/MapRingBuffer.h"
#include <arise_slam_mid360_msgs/msg/laser_feature.hpp>
#include "arise_slam_mid360/config/parameter.h"
#include "arise_slam_mid360/LidarProcess/LidarSlam.h"
#include <std_msgs/msg/string.hpp>


namespace arise_slam {

    struct laser_mapping_config{
        float period;
        float lineRes;
        float planeRes;
        int max_iterations;
        bool debug_view_enabled;
        bool enable_ouster_data;
        bool publish_only_feature_points;
        bool use_imu_roll_pitch;
        // bool start_from_previous_map;
        int max_surface_features;
        double velocity_failure_threshold;
        bool auto_voxel_size;
        bool forget_far_chunks;
        float visual_confidence_factor;
        float pos_degeneracy_threshold;
        float ori_degeneracy_threshold;
        float shift_avg_ratio;
        bool shift_undistortion;
        float yaw_ratio;
        std::string map_dir;
        bool local_mode;
        float init_x;
        float init_y;
        float init_z;
        float init_roll;
        float init_pitch;
        float init_yaw;
        float read_pose_file;
    };
    
    struct OdometryData {
    double timestamp;
    double duration;
    double x, y, z;
    double roll, pitch, yaw;
    };


    class laserMapping : public rclcpp::Node {

    public:
        laserMapping(const rclcpp::NodeOptions & options);

        void initInterface();

        void
        initializationParam();

        void
        preprocessDualLidarFeatures(Transformd current_pose, SensorType sensor_type);

        void
        adjustVoxelSize(int &laserCloudCornerStackNum, int &laserCloudSurfStackNum);

        void
        mappingOptimization(Eigen::Vector3i &postion_in_locamap, Transformd start_tf,tf2::Quaternion roll_pitch_quat,
                            const int laserCloudCornerStackNum, const int laserCloudSurfStackNum);

        void
        publishTopic(Eigen::Vector3i postion_in_locamap);

        void
        transformAssociateToMap(Transformd T_w_pre, Transformd T_wodom_curr, Transformd T_wodom_pre);

        void
        transformAssociateToMap();

        void
        transformUpdate();

        void
        pointAssociateToMap(PointType const *const pi, PointType *const po);

        void
        pointAssociateToMap(pcl::PointXYZHSV const *const pi, pcl::PointXYZHSV *const po);

        void
        pointAssociateTobeMapped(PointType const *const pi, PointType *const po);

        void
        laserFeatureInfoHandler(const arise_slam_mid360_msgs::msg::LaserFeature::SharedPtr msgIn);

        void
        laserCloudRawDataHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudRawdata);

        void
        IMUOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr imuOdometry);

        void
        visualOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr visualOdometry);

        // void 
        // takeoffAlignmentHandler(const takeoff_manager::TakeoffAlignmentConstPtr &msg);

        void
        extractIMUOdometry(double timeLaserFrame, Transformd &T_w_lidar);

        bool
        extractVisualIMUOdometryAndCheck(Transformd &T_w_lidar);

        void
        getOdometryFromTimestamp(MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> &buf, const double &timestamp,
                                 Eigen::Vector3d &T, Eigen::Quaterniond &Q);

        void
        extractRelativeTransform(MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> &buf, Transformd &T_pre_cur, bool imu_prediction);

        void
        setInitialGuess();

        void
        selectposePrediction();

        //TODO: organize the publish topics and odometry
        void
        publishOdometry();

        void
        publishTopic();

        void
        process();

        bool
        readParameters();
        
        bool
        readPointCloud();

        void 
        saveLocalizationPose(double timestamp,Transformd &T_w_lidar, const std::string& parentPath);

        void 
        readLocalizationPose(const std::string& parentPath);

        void
        save_debug_statistic(const std::string file_name);

        template<typename T>
        double secs(T msg) {
            return msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9;
        }

    private:
        static constexpr float vision_laser_time_offset = 0.0;
        static constexpr int laserCloudCenWidth = 10;
        static constexpr int laserCloudCenHeight = 10;
        static constexpr int laserCloudCenDepth = 5;
        static constexpr int laserCloudWidth = 21;
        static constexpr int laserCloudHeight = 21;
        static constexpr int laserCloudDepth = 11;
        static constexpr int laserCloudNum =laserCloudWidth * laserCloudHeight * laserCloudDepth; // 4851

        // ros::NodeHandle *pub_node_;
        // ros::NodeHandle *private_node_;
        
        // ROS
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudCornerLast;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudSurfLast;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subIMUOdometry;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subVisualOdometry;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserRawdata;
        rclcpp::Subscription<arise_slam_mid360_msgs::msg::LaserFeature>::SharedPtr subLaserFeatureInfo;
        // rclcpp::Subscription<>::SharedPtr subTakeoffAlignment;

        // Publisher
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudPrior;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes_rot;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullResOusterWithFeatures;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullResOuster;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudRawRes;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_rot;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMappedHighFrec;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserAfterMappedPath;
        rclcpp::Publisher<arise_slam_mid360_msgs::msg::OptimizationStats>::SharedPtr pubOptimizationStats;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPreviousCloud;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubPreviousPose;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubprediction_source;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubVIOPrediction; 
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLIOPrediction;

        rclcpp::TimerBase::SharedPtr process_timer_;

        rclcpp::CallbackGroup::SharedPtr cb_group_;

        MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> imu_odom_buf;
        MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> visual_odom_buf;

        int frameCount = 0;
        int waiting_takeoff_timeout = 300;
        int startupCount = 10;
        int localizationCount = 0;
        int laserCloudValidInd[125];
        int laserCloudSurroundInd[125];

        double timeLaserCloudCornerLast = 0;
        double timeLaserCloudSurfLast = 0;
        double timeLaserCloudFullRes = 0;
        double timeLaserOdometry = 0;
        double timeLaserOdometryPrev = 0;


        bool got_previous_map = false;
        bool force_initial_guess = false;
        bool odomAvailable = false;
        bool lastOdomAvailable = false;
        bool laser_imu_sync = false;
        bool use_imu_roll_pitch_this_step = false;
        bool initialization = false;
        bool imuodomAvailable = false;
        bool imuorientationAvailable = false;
        bool lastimuodomAvaliable=false;
        bool imu_initialized = false;

        float poseX = 0;
        float poseY = 0;
        float poseZ = 0;

        float shiftX = 0;
        float shiftY = 0;
        float shiftZ = 0;

        pcl::VoxelGrid<PointType> downSizeFilterCorner;
        pcl::VoxelGrid<PointType> downSizeFilterSurf;


        std::queue<sensor_msgs::msg::PointCloud2> cornerLastBuf;
        std::queue<sensor_msgs::msg::PointCloud2> surfLastBuf;
        std::queue<sensor_msgs::msg::PointCloud2> realsenseBuf;
        std::queue<sensor_msgs::msg::PointCloud2> fullResBuf;
        std::queue<sensor_msgs::msg::PointCloud2> rawWithFeaturesBuf;
        std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> rawDataBuf;
        std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf;
        std::queue<Eigen::Quaterniond> IMUPredictionBuf;
        std::queue<SensorType> sensorTypeLastBuf;
        SensorType last_sensor_type_= SensorType::VELODYNE;
     
        
        // variables for stacking 2 scans
        std::queue<pcl::PointCloud<PointType>> cornerDualScanBuf;
        std::queue<pcl::PointCloud<PointType>> surfDualScanBuf;
        std::queue<pcl::PointCloud<pcl::PointXYZHSV>> fullResDualScanBuf;
        std::queue<Transformd> scanTransformBuf;
        std::queue<SensorType> sensorTypeBuf;

        pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
        pcl::PointCloud<PointType>::Ptr laserCloudRealsense;

        pcl::PointCloud<PointType>::Ptr laserCloudSurround;
        pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
        pcl::PointCloud<PointType>::Ptr laserCloudFullRes_rot;
        pcl::PointCloud<PointType>::Ptr laserCloudRawRes;
        pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudRawWithFeatures;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr velodyneLaserCloudRawWithFeatures;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr ousterLaserCloudRawWithFeatures;

        pcl::PointCloud<PointType>::Ptr laserCloudPriorOrg;
        pcl::PointCloud<PointType>::Ptr laserCloudPrior;
        sensor_msgs::msg::PointCloud2 priorCloudMsg;

        Transformd T_w_lidar;
        Transformd last_T_w_lidar;
        Transformd last_imu_T;
        Transformd total_incremental_T;
        Transformd laser_incremental_T;
        Transformd forcedInitialGuess;

        Eigen::Quaterniond q_wmap_wodom;
        Eigen::Vector3d t_wmap_wodom;
        Eigen::Quaterniond q_wodom_curr;
        Eigen::Vector3d t_wodom_curr;
        Eigen::Quaterniond q_wodom_pre;
        Eigen::Vector3d t_wodom_pre;
        Eigen::Quaterniond q_w_imu_pre;
        Eigen::Vector3d t_w_imu_pre;

        LidarSLAM slam;
        laser_mapping_config config_;
        nav_msgs::msg::Path laserAfterMappedPath;
        std::mutex mBuf;
        rclcpp::Time timeLatestImuOdometry;
        rclcpp::Time timeLastMappingResult;
        PointType pointOri, pointSel;

        enum class PredictionSource {IMU_ORIENTATION, IMU_ODOM, VISUAL_ODOM};
        PredictionSource prediction_source;
        std::vector<arise_slam::OdometryData> odometryResults;
           
    }; // class laserMapping

} // namespace arise_slam
#endif //arise_slam_mid360_LASERMAPPING_H
