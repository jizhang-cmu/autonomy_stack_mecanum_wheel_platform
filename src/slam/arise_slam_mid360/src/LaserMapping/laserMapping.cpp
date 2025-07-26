//
// Created by shiboz on 2021-10-18.
//


#include "arise_slam_mid360/LaserMapping/laserMapping.h"

double parameters[7] = {0, 0, 0, 0, 0, 0, 1};
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters);
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters+3);

namespace arise_slam {

    laserMapping::laserMapping(const rclcpp::NodeOptions & options)
    : Node("laser_mapping_node", options) {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
    }

    void laserMapping::initInterface() {
        //! Callback Groups
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

        if(!readGlobalparam(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[AriseSlam::laserMapping] Could not read calibration. Exiting...");
            rclcpp::shutdown();
        }

        if (!readParameters())
        {
            RCLCPP_ERROR(this->get_logger(), "[AriseSlam::laserMapping] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }

        if (!readCalibration(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[AriseSlam::laserMapping] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "DEBUG VIEW: %d", config_.debug_view_enabled);
        RCLCPP_INFO(this->get_logger(), "ENABLE OUSTER DATA: %d", config_.enable_ouster_data);
        RCLCPP_INFO(this->get_logger(), "line resolution %f plane resolution %f vision_laser_time_offset %f",
                config_.lineRes, config_.planeRes, vision_laser_time_offset);

        downSizeFilterCorner.setLeafSize(config_.lineRes, config_.lineRes, config_.lineRes);
        downSizeFilterSurf.setLeafSize(config_.planeRes, config_.planeRes, config_.planeRes);


        subLaserFeatureInfo = this->create_subscription<arise_slam_mid360_msgs::msg::LaserFeature>(
            ProjectName+"/feature_info", 2,
            std::bind(&laserMapping::laserFeatureInfoHandler, this,
                        std::placeholders::_1), sub_options);

        subIMUOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            ProjectName+"/integrated_to_init5", 100,
            std::bind(&laserMapping::IMUOdometryHandler, this,
                        std::placeholders::_1), sub_options);
                        
        subVisualOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "vins_estimator/imu_propagate", 100,
            std::bind(&laserMapping::visualOdometryHandler, this,
                        std::placeholders::_1), sub_options);


        pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/laser_cloud_surround", 2);

        pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/laser_cloud_map", 2);

        pubLaserCloudPrior = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/overall_map", 2);

        pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/registered_scan", 2);

        // pubLaserCloudFullRes_rot = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        //     ProjectName+"/velodyne_cloud_registered", 2);


        // TODO: Remember to add the ouster features
        // pubOdomAftMapped_rot = this->create_publisher<nav_msgs::msg::Odometry>(
        //     ProjectName+"/aft_mapped_to_init", 1);

        pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/laser_odometry", 1);

        pubLaserOdometryIncremental = this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/aft_mapped_to_init_incremental", 1);


        pubVIOPrediction=  this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/vio_prediction", 1);

        pubLIOPrediction= this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/lio_prediction", 1);


        pubLaserAfterMappedPath = this->create_publisher<nav_msgs::msg::Path>(
            ProjectName+"/laser_odom_path", 1);

        pubOptimizationStats = this->create_publisher<arise_slam_mid360_msgs::msg::OptimizationStats>(
            ProjectName+"/arise_slam_mid360_stats", 1);

  

        pubprediction_source = this->create_publisher<std_msgs::msg::String>(
            ProjectName+"/prediction_source", 1);

        process_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(100.)),
            std::bind(&laserMapping::process, this));

        slam.initROSInterface(shared_from_this());
        slam.localMap.lineRes_ = config_.lineRes;
        slam.localMap.planeRes_ = config_.planeRes;
        slam.Visual_confidence_factor=config_.visual_confidence_factor;
        slam.Pos_degeneracy_threshold=config_.pos_degeneracy_threshold;
        slam.Ori_degeneracy_threshold=config_.ori_degeneracy_threshold;
        slam.LocalizationICPMaxIter=config_.max_iterations;
        slam.MaximumSurfFeatures=config_.max_surface_features;
        slam.OptSet.debug_view_enabled=config_.debug_view_enabled;
        slam.OptSet.velocity_failure_threshold=config_.velocity_failure_threshold;
        slam.OptSet.max_surface_features=config_.max_surface_features;
        slam.OptSet.yaw_ratio=yaw_ratio;
        slam.map_dir=config_.map_dir;
        slam.local_mode=config_.local_mode;
        slam.init_x=config_.init_x;
        slam.init_y=config_.init_y;
        slam.init_z=config_.init_z;
        slam.init_roll=config_.init_roll;
        slam.init_pitch=config_.init_pitch;
        slam.init_yaw=config_.init_yaw;

       // slam.localMap = config_.forget_far_chunks;

       // if (config_.debug_view_enabled)
       // {
       // TODO: It will crush since the debug view is not initialized (just for reduce topic publish)
       // slam.debug_view = std::make_shared<arise_slam::DebugView>();
       //   }

        prediction_source = PredictionSource::IMU_ORIENTATION;

        timeLatestImuOdometry = rclcpp::Time(0,0,RCL_ROS_TIME);
        // timeLastMappingResult = rclcpp::Time(0,0,RCL_ROS_TIME);

        initializationParam();

    }

    void laserMapping::initializationParam() {

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurround.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes_rot.reset(new pcl::PointCloud<PointType>());
        laserCloudRawRes.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerStack.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfStack.reset(new pcl::PointCloud<PointType>());
        laserCloudRealsense.reset(new pcl::PointCloud<PointType>());
        laserCloudPriorOrg.reset(new pcl::PointCloud<PointType>());
        laserCloudPrior.reset(new pcl::PointCloud<PointType>());

        Eigen::Quaterniond q_wmap_wodom_(1, 0, 0, 0);
        Eigen::Vector3d t_wmap_wodom_(0, 0, 0);
        Eigen::Quaterniond q_wodom_curr_(1, 0, 0, 0);
        Eigen::Vector3d t_wodom_curr_(0, 0, 0);
        Eigen::Quaterniond q_wodom_pre_(1, 0, 0, 0);
        Eigen::Vector3d t_wodom_pre_(0, 0, 0);

        q_wmap_wodom = q_wmap_wodom_;
        t_wmap_wodom = t_wmap_wodom_;
        q_wodom_curr = q_wodom_curr_;
        t_wodom_curr = t_wodom_curr_;
        q_wodom_pre = q_wodom_pre_;
        t_wodom_pre = t_wodom_pre_;

        imu_odom_buf.allocate(5000);
        visual_odom_buf.allocate(5000);
        
        slam.localMap.setOrigin(Eigen::Vector3d(slam.init_x, slam.init_y, slam.init_z));

        if (slam.local_mode) {
            RCLCPP_INFO(this->get_logger(), "\033[1;32m Loading map....\033[0m");
            if(readPointCloud()) {
                slam.localMap.addSurfPointCloud(*laserCloudPrior);
                RCLCPP_INFO(this->get_logger(), "\033[1;32m Loaded map succesfully, started SLAM in localization mode.\033[0m");
            } else {
                slam.local_mode = false;
                RCLCPP_INFO(this->get_logger(), "\033[1;32mCannot read map file, switched to mapping mode.\033[0m");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "\033[1;32mStarted SLAM in mapping mode.\033[0m");
        }
    }

    bool laserMapping::readParameters()
    {
        this->declare_parameter<float>("mapping_line_resolution", 0.1);
        this->declare_parameter<float>("mapping_plane_resolution", 0.2);
        this->declare_parameter<int>("max_iterations", 4);
        this->declare_parameter<bool>("debug_view", false);
        this->declare_parameter<bool>("enable_ouster_data", false);
        this->declare_parameter<bool>("publish_only_feature_points", false);
        this->declare_parameter<bool>("use_imu_roll_pitch", true);
        this->declare_parameter<int>("max_surface_features", 1000);
        this->declare_parameter<double>("velocity_failure_threshold", 30.0);
        this->declare_parameter<bool>("auto_voxel_size", true);
        this->declare_parameter<bool>("forget_far_chunks", true);
        this->declare_parameter<float>("visual_confidence_factor", 1.0);
        this->declare_parameter<float>("pos_degeneracy_threshold", 1.0);
        this->declare_parameter<float>("ori_degeneracy_threshold", 1.0);
        this->declare_parameter<std::string>("map_dir", "pointcloud_local.txt");
        this->declare_parameter<bool>("local_mode", false);
        this->declare_parameter<float>("init_x", 0.0);
        this->declare_parameter<float>("init_y", 0.0);
        this->declare_parameter<float>("init_z", 0.0);
        this->declare_parameter<float>("init_roll", 0.0);
        this->declare_parameter<float>("init_pitch", 0.0);
        this->declare_parameter<float>("init_yaw", 0.0);
        this->declare_parameter<bool>("read_pose_file", false);

        config_.lineRes = get_parameter("mapping_line_resolution").as_double();
        config_.planeRes = get_parameter("mapping_plane_resolution").as_double();
        config_.max_iterations = get_parameter("max_iterations").as_int();
        config_.debug_view_enabled = get_parameter("debug_view").as_bool();
        config_.enable_ouster_data = get_parameter("enable_ouster_data").as_bool();
        config_.publish_only_feature_points = get_parameter("publish_only_feature_points").as_bool();
        config_.use_imu_roll_pitch = get_parameter("use_imu_roll_pitch").as_bool();
        config_.max_surface_features = get_parameter("max_surface_features").as_int();
        config_.velocity_failure_threshold = get_parameter("velocity_failure_threshold").as_double();
        config_.auto_voxel_size = get_parameter("auto_voxel_size").as_bool();
        config_.forget_far_chunks = get_parameter("forget_far_chunks").as_bool();
        config_.visual_confidence_factor = get_parameter("visual_confidence_factor").as_double();
        config_.pos_degeneracy_threshold = get_parameter("pos_degeneracy_threshold").as_double();
        config_.ori_degeneracy_threshold = get_parameter("ori_degeneracy_threshold").as_double(); 
        config_.map_dir = get_parameter("map_dir").as_string(); 
        config_.local_mode = get_parameter("local_mode").as_bool();
        config_.read_pose_file = get_parameter("read_pose_file").as_bool();

        if(config_.read_pose_file)
        {   
            readLocalizationPose(config_.map_dir);
            config_.init_x= odometryResults[0].x;
            config_.init_y= odometryResults[0].y;
            config_.init_z= odometryResults[0].z;
            config_.init_roll= odometryResults[0].roll;
            config_.init_pitch= odometryResults[0].pitch;
            config_.init_yaw= odometryResults[0].yaw;
        }
        else
        {  
            config_.init_x = get_parameter("init_x").as_double(); 
            config_.init_y = get_parameter("init_y").as_double(); 
            config_.init_z = get_parameter("init_z").as_double(); 
            config_.init_roll = get_parameter("init_roll").as_double();
            config_.init_pitch = get_parameter("init_pitch").as_double();
            config_.init_yaw = get_parameter("init_yaw").as_double(); 
        }

        return true;
    }
    
    bool laserMapping::readPointCloud()
    {
        FILE *map_file = fopen(slam.map_dir.c_str(), "r");
        if (map_file == NULL) {
            return false;
        }
        
        PointType pointRead;
        float intensity, time;
        int val1, val2, val3, val4, val5;
        while (1) {
            val1 = fscanf(map_file, "%f", &pointRead.x);
            val2 = fscanf(map_file, "%f", &pointRead.y);
            val3 = fscanf(map_file, "%f", &pointRead.z);
            val4 = fscanf(map_file, "%f", &intensity);
            val5 = fscanf(map_file, "%f", &time);

            if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) break;
        
            laserCloudPriorOrg->push_back(pointRead);
        }
        
        downSizeFilterSurf.setInputCloud(laserCloudPriorOrg);
        downSizeFilterSurf.filter(*laserCloudPrior);
        laserCloudPriorOrg->clear();
        
        pcl::toROSMsg(*laserCloudPrior, priorCloudMsg);
        priorCloudMsg.header.frame_id = WORLD_FRAME;
        
        return true;
    } 

    void laserMapping::readLocalizationPose(const std::string& parentPath)
    {   
        std::cerr << "Reading localization pose..." << std::endl;
        std::string saveOdomPath;
        size_t lastSlashPos = parentPath.find_last_of('/');
        if (lastSlashPos != std::string::npos) {
            saveOdomPath=parentPath.substr(0, lastSlashPos + 1); // Include the trailing slash
        }
        
        std::string localizationPosePath = saveOdomPath + "start_pose.txt";
        std::ifstream file(localizationPosePath);
        if (!file.is_open()) {
            std::cerr << "Error opening file: " << localizationPosePath << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) {
            continue;
    }
            std::istringstream iss(line);
            arise_slam::OdometryData odom;
            if (iss >> odom.x >> odom.y >> odom.z >> odom.roll >> odom.pitch >> odom.yaw >> odom.duration) {
                std::cout << "Read odometry data: " << odom.x << " " << odom.y << " " << odom.z << std::endl;
                odometryResults.push_back(odom);
            } else {
                std::cerr << "Error reading line: " << line << std::endl;
            }
        }
        std::cerr << "\033[1;32m  loaded the localization_pose.txt successfully \033[0m" <<odometryResults[0].x 
        <<" "<<odometryResults[0].y<<" "<<odometryResults[0].z<< std::endl;
        file.close();
    }

    // Function to save odometry data to a text file
    void laserMapping::saveLocalizationPose(double timestamp,Transformd &T_w_lidar, const std::string& parentPath) {
        
        std::string saveOdomPath;
        size_t lastSlashPos = parentPath.find_last_of('/');
        if (lastSlashPos != std::string::npos) {
            saveOdomPath=parentPath.substr(0, lastSlashPos + 1); // Include the trailing slash
            }

        arise_slam::OdometryData odom;
        {
         odom.timestamp = timestamp;
         odom.x = T_w_lidar.pos.x();
         odom.y = T_w_lidar.pos.y();
         odom.z = T_w_lidar.pos.z(); 
         tf2::Quaternion orientation(T_w_lidar.rot.x(), T_w_lidar.rot.y(), T_w_lidar.rot.z(), T_w_lidar.rot.w());
         tf2::Matrix3x3(orientation).getRPY(odom.roll, odom.pitch, odom.yaw);
        }

        odometryResults.push_back(odom);
        
        std::string OdomResultPath=saveOdomPath+"start_pose.txt";
        std::ofstream outFile(OdomResultPath, std::ios::app);
        
        if (!outFile.is_open()) {
            std::cerr << "Error opening file: " << OdomResultPath << std::endl;
            return;
        }

        outFile << std::fixed << " " << odom.x << " " << odom.y << " " << odom.z << " "
        << odom.roll << " " <<odom.pitch << " " << odom.yaw << odom.timestamp-odometryResults[0].timestamp << std::endl;

        outFile.close();
    }

    void laserMapping::transformAssociateToMap(Transformd T_w_pre, Transformd T_wodom_curr, Transformd T_wodom_pre) {

        Transformd T_wodom_pre_curr = T_wodom_pre.inverse() * T_wodom_curr;

        Transformd T_w_curr_predict = T_w_pre * T_wodom_pre_curr;
        q_w_curr = T_w_curr_predict.rot;
        t_w_curr = T_w_curr_predict.pos;

    }

    void laserMapping::transformAssociateToMap() {
        q_w_curr = q_wmap_wodom * q_wodom_curr;
        t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
    }

    void laserMapping::transformUpdate() {
        q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
        t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;

    }

    void laserMapping::pointAssociateToMap(PointType const *const pi, PointType *const po) {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
        // po->intensity = 1.0;
    }

    void laserMapping::pointAssociateToMap(pcl::PointXYZHSV const *const pi, pcl::PointXYZHSV *const po)
    {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->h = pi->h;
        po->s = pi->s;
        po->v = pi->v;
        // po->intensity = 1.0;
    }

    void laserMapping::pointAssociateTobeMapped(PointType const *const pi, PointType *const po) {
        Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
        po->x = point_curr.x();
        po->y = point_curr.y();
        po->z = point_curr.z();
        po->intensity = pi->intensity;
    }


    void laserMapping::laserFeatureInfoHandler(const arise_slam_mid360_msgs::msg::LaserFeature::SharedPtr msgIn) {
       
        mBuf.lock();
        cornerLastBuf.push(msgIn->cloud_corner);
        surfLastBuf.push(msgIn->cloud_surface);
        realsenseBuf.push(msgIn->cloud_realsense);
        fullResBuf.push(msgIn->cloud_nodistortion);
        Eigen::Quaterniond imuprediction_tmp(msgIn->initial_quaternion_w, msgIn->initial_quaternion_x,
                                             msgIn->initial_quaternion_y, msgIn->initial_quaternion_z);

        IMUPredictionBuf.push(imuprediction_tmp);
        mBuf.unlock();
    }


    void laserMapping::IMUOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr imuOdometry) {
     
        mBuf.lock();
        imu_odom_buf.addMeas(imuOdometry, secs(imuOdometry));
        timeLatestImuOdometry = imuOdometry->header.stamp;
        mBuf.unlock();
    }

    void laserMapping::visualOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr visualOdometry) {
       mBuf.lock();
       visual_odom_buf.addMeas(visualOdometry, secs(visualOdometry));
       mBuf.unlock();
    }

    void laserMapping::setInitialGuess()
    {
        use_imu_roll_pitch_this_step=false;

        if (initialization == false)  //directly hardset the imu rotation as the first pose
        {
            use_imu_roll_pitch_this_step=true;

            if(use_imu_roll_pitch_this_step)
            {
                
                double roll, pitch, yaw;
                tf2::Quaternion orientation_curr(q_wodom_curr.x(), q_wodom_curr.y(), q_wodom_curr.z(), q_wodom_curr.w());
                tf2::Matrix3x3(orientation_curr).getRPY(roll, pitch, yaw);
                RCLCPP_DEBUG(this->get_logger(), "Directly Use IMU Yaw: %f", yaw);
                tf2::Quaternion yaw_quat;
                yaw_quat.setRPY(0, 0, - yaw); //make sure the yaw angle is zero at the beginning
                RCLCPP_DEBUG(this->get_logger(), "Start roll, pitch, yaw %f, %f, %f", roll, pitch, yaw);
                tf2::Quaternion first_orientation;
                first_orientation = yaw_quat*orientation_curr;
                first_orientation = first_orientation;
                q_w_curr = Eigen::Quaterniond(first_orientation.w(), first_orientation.x(), first_orientation.y(), first_orientation.z());
                auto q_extrinsic=Eigen::Quaterniond(imu_laser_R);
                q_extrinsic.normalize();
                RCLCPP_DEBUG(this->get_logger(), "q_extrinsic: %f %f %f %f", q_extrinsic.w(),q_extrinsic.x(), q_extrinsic.y(), q_extrinsic.z());
                RCLCPP_DEBUG(this->get_logger(), "q_w_curr_pre: %f %f %f %f", q_w_curr.w(),q_w_curr.x(), q_w_curr.y(), q_w_curr.z() );

                q_w_curr = q_extrinsic.inverse()*q_w_curr;
                RCLCPP_DEBUG(this->get_logger(), "q_w_curr_pre: %f %f %f %f", q_w_curr.w(),q_w_curr.x(), q_w_curr.y(), q_w_curr.z() );
                //q_w_curr = q_wodom_curr;
                q_wodom_pre = q_w_curr;
                T_w_lidar.rot=q_w_curr;
                
                if(slam.local_mode)
                {
                    
                T_w_lidar.pos=Eigen::Vector3d(slam.init_x, slam.init_y, slam.init_z);
                tf2::Quaternion quat ;
                quat.setRPY(slam.init_roll,slam.init_pitch, slam.init_yaw);
                T_w_lidar.rot=Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
                RCLCPP_DEBUG(this->get_logger(), "\033[1;32m  Localization Mode: x: %f y: %f z: %f roll: %f pitch: %f yaw:%f \033[0m",
                            slam.init_x,slam.init_y,slam.init_z,slam.init_roll, slam.init_pitch, slam.init_yaw);
                slam.last_T_w_lidar=T_w_lidar;
                }

                //  initialization = true;

            }else
            {   
               
                RCLCPP_WARN(this->get_logger(), "start from zero");
                q_w_curr = Eigen::Quaterniond(cos(slam.init_yaw / 2), 0, 0, sin(slam.init_yaw / 2));
                q_wodom_pre = Eigen::Quaterniond(cos(slam.init_yaw / 2), 0, 0, sin(slam.init_yaw / 2));
                T_w_lidar.rot=q_w_curr;
                if(slam.local_mode)
                { 
                  T_w_lidar.pos=Eigen::Vector3d(slam.init_x, slam.init_y, slam.init_z);
                  tf2::Quaternion quat ;
                  quat.setRPY(slam.init_roll,slam.init_pitch, slam.init_yaw);
                  T_w_lidar.rot=Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
                
                }
            }

        } else if (startupCount>0) // To use IMU orientation for a while for initialization
        {

            RCLCPP_WARN(this->get_logger(), "localization/startup");
            use_imu_roll_pitch_this_step = true;
            selectposePrediction();
            if(use_imu_roll_pitch_this_step){
                q_w_curr = T_w_lidar.rot;
            }else{
                q_w_curr = last_T_w_lidar.rot;
            }

            t_w_curr = last_T_w_lidar.pos;
            T_w_lidar.pos = t_w_curr;
            T_w_lidar.rot = q_w_curr;
            startupCount--;
        }
        else
        {   
          
            if (config_.use_imu_roll_pitch)
                use_imu_roll_pitch_this_step=true;
            selectposePrediction();
            q_w_curr = T_w_lidar.rot;
            t_w_curr = T_w_lidar.pos;
        }

    }

    void laserMapping::selectposePrediction()
    {
        prediction_source =PredictionSource::IMU_ODOM; 
        odomAvailable = extractVisualIMUOdometryAndCheck(T_w_lidar);
        if (odomAvailable) {
            return;
        } else {
            // use the incremental imu orientation as the initial guess
            if (imuorientationAvailable == true)
            {
                //RCLCPP_WARN(this->get_logger(), "Using IMU orientation as initial guess");
                //TODO: Need to use the absoulte orientation as the fist rotation instead of q(1,0,0,0),
                //therefore, the system didn't support the gravity alligned
                //TODO: Debug purposes 
                
                Eigen::Quaterniond q_w_predict = q_w_curr * q_wodom_pre.inverse() * q_wodom_curr;
                // Eigen::Quaterniond q_w_predict = q_w_curr;
                q_w_predict.normalize();
                prediction_source =PredictionSource::IMU_ORIENTATION;
                T_w_lidar.rot = q_w_predict;
                q_wodom_pre = q_wodom_curr;
                return;
            }
        }
    }

    void laserMapping::getOdometryFromTimestamp(MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> &buf, const double &timestamp,
                                                Eigen::Vector3d &T, Eigen::Quaterniond &Q) {
        double t_b_i = timestamp;
        auto after_ptr = buf.measMap_.upper_bound(t_b_i);
        //TODO: add checker for before_ptr--
        if (after_ptr->first < 0.001)
        {
            after_ptr = buf.measMap_.begin();
        }

        if (after_ptr == buf.measMap_.begin())
        {
            Q.x()= after_ptr->second->pose.pose.orientation.x;
            Q.y()= after_ptr->second->pose.pose.orientation.y;
            Q.z()= after_ptr->second->pose.pose.orientation.z;
            Q.w()= after_ptr->second->pose.pose.orientation.x;

            T.x() = after_ptr->second->pose.pose.position.x;
            T.y() = after_ptr->second->pose.pose.position.y;
            T.z() = after_ptr->second->pose.pose.position.z;

        }else
        {

            auto before_ptr = after_ptr;
            before_ptr--;

            double ratio_bi =
                    (t_b_i - before_ptr->first) / (after_ptr->first - before_ptr->first);

            Eigen::Quaterniond q_w_i_before;
            q_w_i_before.x() = before_ptr->second->pose.pose.orientation.x;
            q_w_i_before.y() = before_ptr->second->pose.pose.orientation.y;
            q_w_i_before.z() = before_ptr->second->pose.pose.orientation.z;
            q_w_i_before.w() = before_ptr->second->pose.pose.orientation.w;

            Eigen::Quaterniond q_w_i_after;
            q_w_i_after.x() = after_ptr->second->pose.pose.orientation.x;
            q_w_i_after.y() = after_ptr->second->pose.pose.orientation.y;
            q_w_i_after.z() = after_ptr->second->pose.pose.orientation.z;
            q_w_i_after.w() = after_ptr->second->pose.pose.orientation.w;
            Q = q_w_i_before.slerp(ratio_bi, q_w_i_after);

            Eigen::Vector3d t_w_i_before;
            t_w_i_before.x() = before_ptr->second->pose.pose.position.x;
            t_w_i_before.y() = before_ptr->second->pose.pose.position.y;
            t_w_i_before.z() = before_ptr->second->pose.pose.position.z;

            Eigen::Vector3d t_w_i_after;
            t_w_i_after.x() = after_ptr->second->pose.pose.position.x;
            t_w_i_after.y() = after_ptr->second->pose.pose.position.y;
            t_w_i_after.z() = after_ptr->second->pose.pose.position.z;

            T = (1 - ratio_bi) * t_w_i_before + ratio_bi * t_w_i_after;

        }

    }

    void laserMapping::extractRelativeTransform(MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> &buf, Transformd &T_pre_cur, bool imu_prediction) {

        if (lastOdomAvailable == false)
        {
            lastOdomAvailable = true;
         //   timeLaserOdometryPrev = timeLaserOdometry;
        }
       else
       {

            Eigen::Vector3d t_w_imu_cur;
            Eigen::Quaterniond q_w_imu_cur;
            getOdometryFromTimestamp(buf, timeLaserOdometry, t_w_imu_cur, q_w_imu_cur);
            Eigen::Vector3d t_w_imu_pre;
            Eigen::Quaterniond q_w_imu_pre;
            getOdometryFromTimestamp(buf, timeLaserOdometryPrev, t_w_imu_pre, q_w_imu_pre);
            
            // TODO: T_w_imu is actually T_w_lidar from vins-mono
            Transformd T_w_imu_cur(q_w_imu_cur, t_w_imu_cur);
            Transformd T_w_imu_pre(q_w_imu_pre, t_w_imu_pre);
            Transformd T_w_imu_pre_cur = T_w_imu_pre.inverse() * T_w_imu_cur;

            double roll, pitch, yaw;
            tf2::Quaternion orientation_curr(T_w_imu_pre_cur.rot.x(),T_w_imu_pre_cur.rot.y(), T_w_imu_pre_cur.rot.z(), T_w_imu_pre_cur.rot.w());
            tf2::Matrix3x3(orientation_curr).getRPY(roll, pitch, yaw);

            T_pre_cur=T_w_imu_pre_cur;  

            if(imu_prediction==false)
            {
            // RCLCPP_INFO(this->get_logger(), "\033[1;32m----> VIO prediction delta pos x: %f, y: %f, z: %f.\033[0m",T_w_imu_pre_cur.pos[0],T_w_imu_pre_cur.pos[1],T_w_imu_pre_cur.pos[2]);
            // RCLCPP_INFO(this->get_logger(), "FE Start roll, pitch, yaw %f, %f, %f", roll, pitch, yaw);
            nav_msgs::msg::Odometry relativeOdom;
            relativeOdom.header.frame_id = WORLD_FRAME_ROT;
            relativeOdom.child_frame_id =  SENSOR_FRAME_ROT;
            relativeOdom.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
            relativeOdom.pose.pose.orientation.x = T_w_imu_pre_cur.rot.x();
            relativeOdom.pose.pose.orientation.y = T_w_imu_pre_cur.rot.y();
            relativeOdom.pose.pose.orientation.z = T_w_imu_pre_cur.rot.z();
            relativeOdom.pose.pose.orientation.w = T_w_imu_pre_cur.rot.w();
            relativeOdom.pose.pose.position.x = T_w_imu_pre_cur.pos.x();
            relativeOdom.pose.pose.position.y = T_w_imu_pre_cur.pos.y();
            relativeOdom.pose.pose.position.z = T_w_imu_pre_cur.pos.z();
            pubVIOPrediction->publish(relativeOdom);
            }else
            {  
            //RCLCPP_INFO(this->get_logger(), "\033[1;32m----> LIO prediction delta pos x: %f, y: %f, z: %f.\033[0m",T_w_imu_pre_cur.pos[0],T_w_imu_pre_cur.pos[1],T_w_imu_pre_cur.pos[2]);
            //RCLCPP_INFO(this->get_logger(), "FE Start roll, pitch, yaw %f, %f, %f", roll, pitch, yaw);
            nav_msgs::msg::Odometry relativeOdom;
            relativeOdom.header.frame_id = WORLD_FRAME_ROT;
            relativeOdom.child_frame_id =  SENSOR_FRAME_ROT;
            relativeOdom.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
            relativeOdom.pose.pose.orientation.x = T_w_imu_pre_cur.rot.x();
            relativeOdom.pose.pose.orientation.y = T_w_imu_pre_cur.rot.y();
            relativeOdom.pose.pose.orientation.z = T_w_imu_pre_cur.rot.z();
            relativeOdom.pose.pose.orientation.w = T_w_imu_pre_cur.rot.w();
            relativeOdom.pose.pose.position.x = T_w_imu_pre_cur.pos.x();
            relativeOdom.pose.pose.position.y = T_w_imu_pre_cur.pos.y();
            relativeOdom.pose.pose.position.z = T_w_imu_pre_cur.pos.z();
            pubLIOPrediction->publish(relativeOdom);
            } 
          // timeLaserOdometryPrev = timeLaserOdometry;
       }
    }

    bool laserMapping::extractVisualIMUOdometryAndCheck(Transformd &T_w_lidar)
    {
        // Assuming VIO won't be available if IMU odom is not available
        if (imu_odom_buf.empty()) {
            // RCLCPP_DEBUG_STREAM(this->get_logger(), "IMU Odometry buffer empty! Skip odometry initial guess");
            return false;
        }

        double imu_odom_first_time, imu_odom_last_time;
        imu_odom_buf.getFirstTime(imu_odom_first_time);
        imu_odom_buf.getLastTime(imu_odom_last_time);

        if ((imu_odom_first_time > timeLaserOdometry) || (imu_odom_last_time < timeLaserOdometry)) {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "IMU Odometry buffer time not covering laser mapping. Skip odometry initial guess");
            return false;
        }
        
        
        if (slam.isDegenerate) {
            //check if VIO available
            bool vio_buf_ready = true;
            if (visual_odom_buf.empty()) {
                RCLCPP_DEBUG_STREAM(this->get_logger(), "Visual Odometry buffer empty.");
                vio_buf_ready = false;
            }

            if (vio_buf_ready) {
                double visual_odom_first_time, visual_odom_last_time;
                visual_odom_buf.getFirstTime(visual_odom_first_time);
                visual_odom_buf.getLastTime(visual_odom_last_time);
                if ((visual_odom_first_time > timeLaserOdometryPrev) || (visual_odom_last_time < timeLaserOdometry)) {
                    RCLCPP_DEBUG_STREAM(this->get_logger(), "Visual Odometry buffer time not covering laser mapping.");
                    vio_buf_ready = false;
                }
            }

            if (vio_buf_ready) {

                double t_b_i = timeLaserOdometry;
                auto after_ptr = visual_odom_buf.measMap_.upper_bound(t_b_i);
                //TODO: add checker for before_ptr--
             
                if (after_ptr->first < 0.001)
                  {
                        after_ptr = visual_odom_buf.measMap_.begin();
                        vio_buf_ready = false;
                   }else
                    {
                    auto before_ptr = after_ptr;     
                    before_ptr--;
                    auto after_init_stat = int(after_ptr->second->pose.covariance[1]);
                    auto before_init_stat = int(before_ptr->second->pose.covariance[1]);
                    auto after_failure_stat = int(after_ptr->second->pose.covariance[0]);
                    auto before_failure_stat = int(before_ptr->second->pose.covariance[0]);
                        if (after_init_stat != 1 || before_init_stat != 1 || after_failure_stat!=0 || before_failure_stat!=0) {
                            RCLCPP_WARN_STREAM(this->get_logger(), "Visual Odometry current buffer not initialized.");
                            vio_buf_ready = false;
                        }
                    }

                    double t_b_prev = timeLaserOdometryPrev;
                    auto after_ptr_prev = visual_odom_buf.measMap_.upper_bound(t_b_prev);
                    
                    if (after_ptr_prev->first < 0.001)
                    {
                        after_ptr_prev = visual_odom_buf.measMap_.begin();
                        vio_buf_ready = false;
                    }else
                    {                   
                        //TODO: add checker for before_ptr--
                        auto before_ptr_prev = after_ptr_prev;
                        before_ptr_prev--;
                        auto after_init_stat_prev = int(after_ptr_prev->second->pose.covariance[1]);
                        auto before_init_stat_prev = int(before_ptr_prev->second->pose.covariance[1]); 
                        auto after_failure_stat_prev = int(after_ptr_prev->second->pose.covariance[0]);
                        auto before_failure_stat_prev = int(before_ptr_prev->second->pose.covariance[0]);  
                        if (after_init_stat_prev != 1 || before_init_stat_prev != 1 ||after_failure_stat_prev!=0|| before_failure_stat_prev!=0) {
                            RCLCPP_WARN_STREAM(this->get_logger(), "Visual Odometry prev buffer not initialized.");
                            vio_buf_ready = false;
                        }
                    }

            }

            if (vio_buf_ready) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 10.0, 
                    "VIO is ready for predition.");
                prediction_source = PredictionSource::VISUAL_ODOM;
                Transformd T_pre_cur;
                extractRelativeTransform(visual_odom_buf, T_pre_cur,false);
               // extractRelativeTransform(imu_odom_buf, T_pre_cur,true);
                T_w_lidar = T_w_lidar * T_pre_cur;

               if(use_imu_roll_pitch_this_step)
               {
                Eigen::Quaterniond q_w_imu_cur;
                Eigen::Vector3d t_w_imu_cur;
                getOdometryFromTimestamp(imu_odom_buf, timeLaserOdometry, t_w_imu_cur, q_w_imu_cur);   
                T_w_lidar.rot=q_w_imu_cur;
               }

                return true;
            }
        }

        prediction_source = PredictionSource::IMU_ODOM;
        Transformd T_pre_cur;
        extractRelativeTransform(imu_odom_buf, T_pre_cur,true);
        T_w_lidar = T_w_lidar * T_pre_cur;
        
        if(use_imu_roll_pitch_this_step)
        {
          Eigen::Quaterniond q_w_imu_cur;
          Eigen::Vector3d t_w_imu_cur;
          getOdometryFromTimestamp(imu_odom_buf, timeLaserOdometry, t_w_imu_cur, q_w_imu_cur);   
          T_w_lidar.rot=q_w_imu_cur;
        }
        return true;
    }

    void laserMapping::publishTopic(){

        TicToc t_pub;
        std_msgs::msg::String prediction_source_msg;
        switch (prediction_source) {
            case PredictionSource::IMU_ORIENTATION :
                prediction_source_msg.data = "IMU Only Orientation Prediction";
                break;
            case PredictionSource::IMU_ODOM :
                prediction_source_msg.data = "Using Laser-Inertial Odometry (LIO)";
                break;
            case PredictionSource::VISUAL_ODOM :
                prediction_source_msg.data = "Using Visual-Inertial Odometry (VIO)";
                break;
        }
        pubprediction_source->publish(prediction_source_msg);

        if (frameCount % 5 == 0 && config_.debug_view_enabled) {
            laserCloudSurround->clear();
            *laserCloudSurround = slam.localMap.get5x5LocalMap(slam.pos_in_localmap);
            sensor_msgs::msg::PointCloud2 laserCloudSurround3;
            pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
            laserCloudSurround3.header.stamp =
                    rclcpp::Time(timeLaserOdometry*1e9);
            laserCloudSurround3.header.frame_id = WORLD_FRAME;
            pubLaserCloudSurround->publish(laserCloudSurround3);
        }

        if (frameCount % 20 == 0) {
            pcl::PointCloud<PointType> laserCloudMap;
            laserCloudMap = slam.localMap.getAllLocalMap();
            sensor_msgs::msg::PointCloud2 laserCloudMsg;
            pcl::toROSMsg(laserCloudMap, laserCloudMsg);
            laserCloudMsg.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
            laserCloudMsg.header.frame_id = WORLD_FRAME;
            pubLaserCloudMap->publish(laserCloudMsg);
            
            if (slam.local_mode) {
                priorCloudMsg.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
                pubLaserCloudPrior->publish(priorCloudMsg);
            }
        }

        int laserCloudFullResNum = laserCloudFullRes->points.size();
        for (int i = 0; i < laserCloudFullResNum; i++) {
            PointType const *const &pi = &laserCloudFullRes->points[i];
            if (pi->x* pi->x+ pi->y * pi->y + pi->z* pi->z < 0.01)
            {
                continue;
            }
            pointAssociateToMap(&laserCloudFullRes->points[i],
                                &laserCloudFullRes->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
        laserCloudFullRes3.header.frame_id = WORLD_FRAME;
        pubLaserCloudFullRes->publish(laserCloudFullRes3);

        laserCloudFullRes_rot->clear();
        laserCloudFullRes_rot->resize(laserCloudFullResNum);

        for (int i = 0; i < laserCloudFullResNum; i++) {
            laserCloudFullRes_rot->points[i].x = laserCloudFullRes->points[i].y;
            laserCloudFullRes_rot->points[i].y = laserCloudFullRes->points[i].z;
            laserCloudFullRes_rot->points[i].z = laserCloudFullRes->points[i].x;
            laserCloudFullRes_rot->points[i].intensity = laserCloudFullRes->points[i].intensity;
        }

        // sensor_msgs::msg::PointCloud2 laserCloudFullRes4;
        // pcl::toROSMsg(*laserCloudFullRes_rot, laserCloudFullRes4);
        // laserCloudFullRes4.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
        // laserCloudFullRes4.header.frame_id = WORLD_FRAME_ROT;
        // pubLaserCloudFullRes_rot->publish(laserCloudFullRes4);


        // covert to loam axis
        // nav_msgs::msg::Odometry odomAftMapped_rot;
        // odomAftMapped_rot.header.frame_id = WORLD_FRAME_ROT;
        // odomAftMapped_rot.child_frame_id =  SENSOR_FRAME_ROT;
        // odomAftMapped_rot.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);

        // odomAftMapped_rot.pose.pose.orientation.x = q_w_curr.y();
        // odomAftMapped_rot.pose.pose.orientation.y = q_w_curr.z();
        // odomAftMapped_rot.pose.pose.orientation.z = q_w_curr.x();
        // odomAftMapped_rot.pose.pose.orientation.w = q_w_curr.w();

        // odomAftMapped_rot.pose.pose.position.x = t_w_curr.y();
        // odomAftMapped_rot.pose.pose.position.y = t_w_curr.z();
        // odomAftMapped_rot.pose.pose.position.z = t_w_curr.x();
        // pubOdomAftMapped_rot->publish(odomAftMapped_rot);

        nav_msgs::msg::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = WORLD_FRAME;
        odomAftMapped.child_frame_id = SENSOR_FRAME;
        odomAftMapped.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);

        odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
        odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
        odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
        odomAftMapped.pose.pose.orientation.w = q_w_curr.w();

        odomAftMapped.pose.pose.position.x = t_w_curr.x();
        odomAftMapped.pose.pose.position.y = t_w_curr.y();
        odomAftMapped.pose.pose.position.z = t_w_curr.z();

        nav_msgs::msg::Odometry laserOdomIncremental;

        if (initialization == false)
        {
            laserOdomIncremental.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
            laserOdomIncremental.header.frame_id = WORLD_FRAME;
            laserOdomIncremental.child_frame_id =  SENSOR_FRAME;
            laserOdomIncremental.pose.pose.position.x = t_w_curr.x();
            laserOdomIncremental.pose.pose.position.y = t_w_curr.y();
            laserOdomIncremental.pose.pose.position.z = t_w_curr.z();
            laserOdomIncremental.pose.pose.orientation.x = q_w_curr.x();
            laserOdomIncremental.pose.pose.orientation.y = q_w_curr.y();
            laserOdomIncremental.pose.pose.orientation.z = q_w_curr.z();
            laserOdomIncremental.pose.pose.orientation.w = q_w_curr.w();
        }
        else
        {

            laser_incremental_T = T_w_lidar;
            laser_incremental_T.rot.normalized();

            laserOdomIncremental.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
            laserOdomIncremental.header.frame_id = WORLD_FRAME;
            laserOdomIncremental.child_frame_id =  SENSOR_FRAME;
            laserOdomIncremental.pose.pose.position.x = laser_incremental_T.pos.x();
            laserOdomIncremental.pose.pose.position.y = laser_incremental_T.pos.y();
            laserOdomIncremental.pose.pose.position.z = laser_incremental_T.pos.z();
            laserOdomIncremental.pose.pose.orientation.x = laser_incremental_T.rot.x();
            laserOdomIncremental.pose.pose.orientation.y = laser_incremental_T.rot.y();
            laserOdomIncremental.pose.pose.orientation.z = laser_incremental_T.rot.z();
            laserOdomIncremental.pose.pose.orientation.w = laser_incremental_T.rot.w();
        }

        pubLaserOdometryIncremental->publish(laserOdomIncremental);


        if (slam.isDegenerate) {
            odomAftMapped.pose.covariance[0] = 1;
        } else {
            odomAftMapped.pose.covariance[0] = 0;
        }

       // RCLCPP_INFO_STREAM(this->get_logger(), " slam.isDegenerate: " <<  slam.isDegenerate);
        rclcpp::Time pub_time = rclcpp::Clock{RCL_ROS_TIME}.now(); //PARV_TODO - find how to syncrynoise this with rosbag time
        pubOdomAftMapped->publish(odomAftMapped);

        geometry_msgs::msg::PoseStamped laserAfterMappedPose;
        laserAfterMappedPose.header = odomAftMapped.header;
        laserAfterMappedPose.pose = odomAftMapped.pose.pose;
        laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
        laserAfterMappedPath.header.frame_id = WORLD_FRAME;
        laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
        pubLaserAfterMappedPath->publish(laserAfterMappedPath);


        slam.stats.header = odomAftMapped.header;
        if (timeLatestImuOdometry.seconds() < 1.0)
        {
            timeLatestImuOdometry = pub_time;
        }
        rclcpp::Duration latency = timeLatestImuOdometry - pub_time;  
        slam.stats.latency = latency.seconds() * 1000;
        slam.stats.n_iterations = slam.stats.iterations.size();
        // Avoid breaking rqt_multiplot
        while (slam.stats.iterations.size() < 4)
        {
            slam.stats.iterations.push_back(arise_slam_mid360_msgs::msg::IterationStats());
        }

        pubOptimizationStats->publish(slam.stats);
        slam.stats.iterations.clear();

    

        // tf2_ros::TransformBroadcaster br(this);
        // // tf2::Transform transform;
        // geometry_msgs::msg::TransformStamped transform_stamped_;
        // tf2::Transform transform;
        // transform_stamped_.header.stamp  = odomAftMapped.header.stamp;
        // transform_stamped_.header.frame_id = WORLD_FRAME;
        // transform_stamped_.child_frame_id = SENSOR_FRAME;
        // tf2::Quaternion q;
        // transform.setOrigin(tf2::Vector3(t_w_curr(0), t_w_curr(1), t_w_curr(2)));
        // q.setW(q_w_curr.w());
        // q.setX(q_w_curr.x());
        // q.setY(q_w_curr.y());
        // q.setZ(q_w_curr.z());
        // transform.setRotation(q);
        // transform_stamped_.transform = tf2::toMsg(transform);
        // br.sendTransform(transform_stamped_);
    }

    void laserMapping::save_debug_statistic (const std::string file_name)
    {

        slam.kdtree_time_analysis.frameID=frameCount;
        slam.kdtree_time_analysis.timestamp=timeLaserOdometry;

        std::cout << std::endl
             << "Saving imporant statistic to " << file_name << " ..." << std::endl;
        std::ofstream f;
        f.open(file_name.c_str(), std::ios::out | std::ios::app);
        f << std::fixed;

        double timestamp;
        int frameID;
        float kdtree_build_time;
        float kdtree_query_time;
        timestamp=timeLaserOdometry;
        frameID=frameCount;
        kdtree_build_time=slam.kdtree_time_analysis.kd_tree_building_time;
        kdtree_query_time=slam.kdtree_time_analysis.kd_tree_query_time;

        f <<frameID<<" "<<std::setprecision(6) << timestamp << " "<<kdtree_build_time<<" "<<kdtree_query_time<< std::endl;

        f.close();
    }


    void laserMapping::adjustVoxelSize(int &laserCloudCornerStackNum, int &laserCloudSurfStackNum){

        // Calculate cloud statistics
        bool increase_blind_radius = false;
        if(config_.auto_voxel_size)
        {
            Eigen::Vector3f average(0,0,0);
            int count_far_points = 0;
           // RCLCPP_INFO(this->get_logger(), "Asurface size: %zu", laserCloudSurfLast->points.size());
            for (auto &point : *laserCloudSurfLast)
            {
                average(0) += fabs(point.x);
                average(1) += fabs(point.y);
                average(2) += fabs(point.z);
                if(point.x*point.x + point.y*point.y + point.z*point.z>9){
                    count_far_points++;
                }
            }
           // RCLCPP_INFO(this->get_logger(), "count_far_points: %d", count_far_points);
            if (count_far_points > 3000)
            {
                increase_blind_radius = true;
            }

            average /= laserCloudSurfLast->points.size();
            // RCLCPP_INFO_STREAM(this->get_logger(), "average: " << average);
            slam.stats.average_distance = average(0)*average(1)*average(2);
            if (slam.stats.average_distance < 25)
            {
              //  RCLCPP_DEBUG(this->get_logger(), "tiny area");
                config_.lineRes = 0.1;
                config_.planeRes = 0.2;
            }
            else if (slam.stats.average_distance > 65)
            {
                //RCLCPP_DEBUG(this->get_logger(), "large area");
                config_.lineRes = 0.4;
                config_.planeRes = 0.8;
            }
            downSizeFilterSurf.setLeafSize(config_.planeRes , config_.planeRes , config_.planeRes );
            downSizeFilterCorner.setLeafSize(config_.lineRes , config_.lineRes , config_.lineRes );
        }

#if 0
        if(increase_blind_radius)
        {
            RCLCPP_DEBUG(this->get_logger(), "increase blind radius");
            pcl::CropBox<PointType> boxFilter;
            float min = -2.0;
            float max = -min;
            boxFilter.setMin(Eigen::Vector4f(min, min, min, 1.0));
            boxFilter.setMax(Eigen::Vector4f(max, max, max, 1.0));
            boxFilter.setNegative(true);
            boxFilter.setInputCloud(laserCloudCornerLast);
            boxFilter.filter(*laserCloudCornerLast);
            boxFilter.setInputCloud(laserCloudSurfLast);
            boxFilter.filter(*laserCloudSurfLast);

            RCLCPP_DEBUG(this->get_logger(), "surface size: %zu", laserCloudSurfLast->points.size());
            RCLCPP_DEBUG(this->get_logger(), "corner size: %zu", laserCloudCornerLast->points.size());
        }
#endif 

        laserCloudCornerStack->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerStack);
        laserCloudCornerStackNum = laserCloudCornerStack->points.size();

        laserCloudSurfStack->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfStack);
        laserCloudSurfStackNum = laserCloudSurfStack->points.size();

        slam.localMap.lineRes_ = config_.lineRes;
        slam.localMap.planeRes_ = config_.planeRes;


    }

    void laserMapping::process() {
        if (rclcpp::ok()) {
            while (!cornerLastBuf.empty() && !surfLastBuf.empty() &&
                   !fullResBuf.empty()&& !IMUPredictionBuf.empty() ) {

                laser_imu_sync = false;

                mBuf.lock();

                timeLaserCloudCornerLast = secs(&cornerLastBuf.front());
                timeLaserCloudSurfLast = secs(&surfLastBuf.front());
                timeLaserCloudFullRes = secs(&fullResBuf.front());
                timeLaserOdometry = timeLaserCloudFullRes;

                laserCloudCornerLast->clear();
                pcl::fromROSMsg(cornerLastBuf.front(), *laserCloudCornerLast);
                cornerLastBuf.pop();

                laserCloudSurfLast->clear();
                pcl::fromROSMsg(surfLastBuf.front(), *laserCloudSurfLast);
                surfLastBuf.pop();

                laserCloudFullRes->clear();
                pcl::fromROSMsg(fullResBuf.front(), *laserCloudFullRes);
                fullResBuf.pop();

                if(!realsenseBuf.empty())
                {
                    laserCloudRealsense->clear();
                    pcl::fromROSMsg(realsenseBuf.front(),*laserCloudRealsense);
                    realsenseBuf.pop();
                }


                Eigen::Quaterniond IMUPrediction;
                IMUPrediction = IMUPredictionBuf.front();
                IMUPredictionBuf.pop();
                IMUPrediction.normalize();

                q_wodom_curr.x() = IMUPrediction.x();
                q_wodom_curr.y() = IMUPrediction.y();
                q_wodom_curr.z() = IMUPrediction.z();
                q_wodom_curr.w() = IMUPrediction.w();
              
                imuorientationAvailable=true;

                setInitialGuess();
                Transformd start_tf(T_w_lidar);

                
                while(!cornerLastBuf.empty())
                {

                    // RCLCPP_DEBUG_STREAM(this->get_logger(), "cornerLastBuf:" << cornerLastBuf.size());
                    cornerLastBuf.pop();
                }

                while(!surfLastBuf.empty())
                {

                    // RCLCPP_DEBUG_STREAM(this->get_logger(), "surfLastBuf:" << surfLastBuf.size());
                    surfLastBuf.pop();
                }

                while(!fullResBuf.empty())
                {

                    // RCLCPP_DEBUG_STREAM(this->get_logger(), "fullResBuf:" << fullResBuf.size());
                    fullResBuf.pop();
                }


                while(!IMUPredictionBuf.empty())
                {

                    // RCLCPP_DEBUG_STREAM(this->get_logger(), "IMUPredictionBuf:" << IMUPredictionBuf.size());
                    IMUPredictionBuf.pop();
                }

                while (!realsenseBuf.empty())
                {

                    // RCLCPP_DEBUG_STREAM(this->get_logger(), "fullResBuf:" << fullResBuf.size());
                    realsenseBuf.pop();
                }

                mBuf.unlock();
                TicToc t_whole;
                int laserCloudCornerStackNum=0;
                int laserCloudSurfStackNum=0;
                adjustVoxelSize(laserCloudCornerStackNum, laserCloudSurfStackNum);

                if (!laserCloudRealsense->empty())
                {
                    *laserCloudSurfStack = *laserCloudSurfStack + *laserCloudRealsense;
                    laserCloudSurfStackNum = laserCloudSurfStack->points.size();
                }

                // RCLCPP_DEBUG(this->get_logger(), "extra surface features  size: %zu", laserCloudRealsense->size());
                // RCLCPP_DEBUG(this->get_logger(), "after surface features  size: %zu", laserCloudSurfStack->size());

                 tf2::Quaternion imu_roll_pitch;
                if (use_imu_roll_pitch_this_step)
                {
                    double imu_roll, imu_pitch, imu_yaw;
                    tf2::Quaternion orientation(T_w_lidar.rot.x(), T_w_lidar.rot.y(), T_w_lidar.rot.z(), T_w_lidar.rot.w());
                    tf2::Matrix3x3(orientation).getRPY(imu_roll, imu_pitch, imu_yaw);
                    RCLCPP_INFO(this->get_logger(), "Using IMU Roll Pitch in ICP: %f %f %f", imu_roll, imu_pitch, imu_yaw);
                    imu_roll_pitch.setRPY(imu_roll, imu_pitch, 0);
                }


                if(prediction_source==PredictionSource::VISUAL_ODOM) {
                     
                    slam.OptSet.use_imu_roll_pitch=use_imu_roll_pitch_this_step;
                    slam.OptSet.imu_roll_pitch=imu_roll_pitch;
                    slam.Localization(initialization, LidarSLAM::PredictionSource::VISUAL_ODOM, T_w_lidar,
                                    laserCloudCornerStack, laserCloudSurfStack,timeLaserOdometry);

                }else {
                          
                    slam.OptSet.use_imu_roll_pitch=use_imu_roll_pitch_this_step;
                    slam.OptSet.imu_roll_pitch=imu_roll_pitch;
                    slam.Localization(initialization, LidarSLAM::PredictionSource::IMU_ODOM, T_w_lidar,
                                laserCloudCornerStack, laserCloudSurfStack,timeLaserOdometry);
                           
                }


                q_w_curr = slam.T_w_lidar.rot;
                t_w_curr = slam.T_w_lidar.pos;

                T_w_lidar.rot=slam.T_w_lidar.rot;
                T_w_lidar.pos=slam.T_w_lidar.pos;

                last_T_w_lidar=slam.T_w_lidar;

                timeLaserOdometryPrev=timeLaserOdometry;
                

                startupCount=slam.startupCount;

                frameCount++;

                slam.frame_count=frameCount;
                slam.laser_imu_sync=laser_imu_sync;

                initialization = true;
#if 0
                LOG(INFO)<<"\033[1;32m Laser observation analysis.\033[0m";
                LOG(INFO)<<"rx_cross: "<<slam.PlaneFeatureHistogramObs.at(0);
                LOG(INFO)<<"neg_rx_cross:"<<slam.PlaneFeatureHistogramObs.at(1);
                LOG(INFO)<<"ry_cross: "<<slam.PlaneFeatureHistogramObs.at(2);
                LOG(INFO)<< "neg_ry_cross: "<<slam.PlaneFeatureHistogramObs.at(3);
                LOG(INFO)<<"rz_cross: "<<slam.PlaneFeatureHistogramObs.at(4);
                LOG(INFO)<<"neg_rz_cross: "<<slam.PlaneFeatureHistogramObs.at(5);
                LOG(INFO)<<"tx_dot: "<<slam.PlaneFeatureHistogramObs.at(6);
                LOG(INFO)<<"ty_dot:"<<slam.PlaneFeatureHistogramObs.at(7);
                LOG(INFO)<<"tz_dot:"<<slam.PlaneFeatureHistogramObs.at(8);
#endif
                // printf("whole mapping time %f ms +++++\n", t_whole.toc());

              
                publishTopic();
                // if(config_.read_pose_file)
                //     saveLocalizationPose(timeLaserOdometry, T_w_lidar, slam.map_dir.c_str());
                // save_debug_statistic(debug_file);
            }
            // std::chrono::milliseconds dura(2);
            // std::this_thread::sleep_for(dura);
        }
    }




#pragma clang diagnostic pop

} // namespace arise_slam
