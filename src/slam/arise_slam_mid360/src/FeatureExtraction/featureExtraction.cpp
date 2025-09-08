//
// Created by shibo zhao on 2020-09-27.
//

#include <arise_slam_mid360/FeatureExtraction/featureExtraction.h>
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

#define BOLD "\033[1m"
#define UNDERLINE "\033[4m"
#define ITALIC "\033[3m"



namespace arise_slam {
    
    featureExtraction::featureExtraction(const rclcpp::NodeOptions & options)
    : Node("feature_extraction_node", options) {
    }

    void featureExtraction::initInterface() {      
        //! Callback Groups
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

        if(!readGlobalparam(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[arise_slam::featureExtraction] Could not read calibration. Exiting...");
            rclcpp::shutdown();
        }
        if (!readParameters())
        {
            RCLCPP_ERROR(this->get_logger(), "[arise_slam::featureExtraction] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "calibration");
        if (!readCalibration(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[arise_slam::featureExtraction] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }
        
        RCLCPP_WARN(this->get_logger(), "config_.skipFrame: %d", config_.skipFrame);
        RCLCPP_WARN(this->get_logger(), "config_.lidar_flip: %d", config_.lidar_flip);
        RCLCPP_INFO(this->get_logger(), "config_.use_imu_roll_pitch %d \n", config_.use_imu_roll_pitch);
        RCLCPP_INFO(this->get_logger(), "scan line number %d \n", config_.N_SCANS);      
        RCLCPP_INFO(this->get_logger(), "\033[1;32m----> use up realsense camera points %d \n \033[0m", config_.use_up_realsense_points);
        RCLCPP_INFO(this->get_logger(), "\033[1;32m----> use down realsense camera points %d \n \033[0m", config_.use_down_realsense_points);

        if (config_.N_SCANS != 16 && config_.N_SCANS != 32 && config_.N_SCANS != 64 && config_.N_SCANS != 4)
        {
            RCLCPP_ERROR(this->get_logger(), "only support velodyne with 16, 32 or 64 scan line! and livox mid 360");
            rclcpp::shutdown();
        }

        // subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     LASER_TOPIC, 2, 
        //     std::bind(&featureExtraction::laserCloudHandler, this,
        //                 std::placeholders::_1), sub_options);

        if (config_.sensor == SensorType::VELODYNE || config_.sensor == SensorType::OUSTER) {
            subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(LASER_TOPIC, 2, 
                    std::bind(&featureExtraction::laserCloudHandler, this,
                    std::placeholders::_1), sub_options);
        } else if (config_.sensor == SensorType::LIVOX) {
            subLivoxCloud = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(LASER_TOPIC, 20, 
                    std::bind(&featureExtraction::livoxHandler, this,
                    std::placeholders::_1), sub_options);
        } //TODO: add this to config

        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, 10, 
            std::bind(&featureExtraction::imu_Handler, this,
                        std::placeholders::_1), sub_options);

        subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
            ODOM_TOPIC, 10, 
            std::bind(&featureExtraction::visual_odom_Handler, this,
                        std::placeholders::_1), sub_options);

        // sub_depth_up = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     DepthUP_TOPIC, 2, 
        //     std::bind(&featureExtraction::depthupHandler, this,
        //                 std::placeholders::_1));

        // sub_depth_down = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     DepthDown_TOPIC, 2, 
        //     std::bind(&featureExtraction::depthdownHandler, this,
        //                 std::placeholders::_1));

        pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/velodyne_cloud_2", 2);

        pubLaserFeatureInfo = this->create_publisher<arise_slam_mid360_msgs::msg::LaserFeature>(
            ProjectName+"/feature_info", 2);

        pubBobPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/realsense_depth_points", 2);

        pubPlannerPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/planner_points", 2);

        pubEdgePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/edge_points", 2);

        // pubDepthUpPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        //     ProjectName+"/depth_up_points", 1);

        // pubDepthDownPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        //     ProjectName+"/depth_down_points", 1);

        KeyPointsExtractor->MinDistanceToSensor = config_.min_range;
        KeyPointsExtractor->MaxDistanceToSensor = config_.max_range;

        if (PUB_EACH_LINE)
        {
            for (int i = 0; i < config_.N_SCANS; i++)
            {
                auto tmp_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "laser_scanid_" + std::to_string(i), 2);
                pubEachScan.push_back(tmp_publisher_);
            }
        }
        delay_count_ = 0;

        m_imuPeriod = 1.0/200;

    }

     void featureExtraction::depthupHandler(const sensor_msgs::msg::PointCloud2::SharedPtr depthupMsg)
    {
        m_buf.lock();
        sensor_msgs::msg::PointCloud2::SharedPtr msg = depthupMsg; // std::make_shared<sensor_msgs::msg::PointCloud2>(*depthupMsg);
        all_cloud_buf[0].push(msg);
        m_buf.unlock();
    }
 
     void featureExtraction::depthdownHandler(const sensor_msgs::msg::PointCloud2::SharedPtr depthdownMsg)
    {
        m_buf.lock();
        sensor_msgs::msg::PointCloud2::SharedPtr msg = depthdownMsg;
        all_cloud_buf[1].push(msg);
        m_buf.unlock();
    }


    // void featureExtraction::depthCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud0_msg,
    //                                            const sensor_msgs::msg::PointCloud2::SharedPtr cloud1_msg)
    // {
    //     m_buf.lock();
    //     all_cloud_buf[0].push(cloud0_msg);
    //     all_cloud_buf[1].push(cloud1_msg);
    //     m_buf.unlock();
    // }

    bool featureExtraction::readParameters()
    {          
        this->declare_parameter<int>("scan_line");
        this->declare_parameter<int>("mapping_skip_frame");
        this->declare_parameter<bool>("lidar_flip");
        this->declare_parameter<double>("blindFront");
        this->declare_parameter<double>("blindBack");
        this->declare_parameter<double>("blindLeft");
        this->declare_parameter<double>("blindRight");
        this->declare_parameter<double>("blindDiskLow");
        this->declare_parameter<double>("blindDiskHigh");
        this->declare_parameter<double>("blindDiskRadius");
        this->declare_parameter<bool>("use_dynamic_mask");
        this->declare_parameter<bool>("use_imu_roll_pitch");
        this->declare_parameter<bool>("use_up_realsense_points");
        this->declare_parameter<bool>("use_down_realsense_points");
        this->declare_parameter<float>("min_range");
        this->declare_parameter<float>("max_range");
        this->declare_parameter<int>("skip_realsense_points");
        this->declare_parameter<int>("provide_point_time");
        this->declare_parameter<std::string>("sensor");
        this->declare_parameter<bool>("debug_view", false);
        this->declare_parameter<double>("imu_acc_x_limit");
        this->declare_parameter<double>("imu_acc_y_limit");
        this->declare_parameter<double>("imu_acc_z_limit");
        
        config_.N_SCANS = this->get_parameter("scan_line").as_int();
        config_.skipFrame = this->get_parameter("mapping_skip_frame").as_int();
        config_.lidar_flip = this->get_parameter("lidar_flip").as_bool();
        config_.box_size.blindFront = this->get_parameter("blindFront").as_double();
        config_.box_size.blindBack = this->get_parameter("blindBack").as_double();
        config_.box_size.blindLeft = this->get_parameter("blindLeft").as_double();
        config_.box_size.blindRight = this->get_parameter("blindRight").as_double();
        config_.box_size.blindDiskLow = this->get_parameter("blindDiskLow").as_double();
        config_.box_size.blindDiskHigh = this->get_parameter("blindDiskHigh").as_double();
        config_.box_size.blindDiskRadius = this->get_parameter("blindDiskRadius").as_double();
        config_.use_imu_roll_pitch = this->get_parameter("use_imu_roll_pitch").as_bool();
        config_.use_up_realsense_points = this->get_parameter("use_up_realsense_points").as_bool();
        config_.use_down_realsense_points = this->get_parameter("use_down_realsense_points").as_bool();       
        config_.min_range = this->get_parameter("min_range").as_double();
        config_.max_range = this->get_parameter("max_range").as_double();
        config_.skip_realsense_points = this->get_parameter("skip_realsense_points").as_int();
        config_.provide_point_time = this->get_parameter("provide_point_time").as_int();
        config_.use_dynamic_mask = this->get_parameter("use_dynamic_mask").as_bool(); 
        config_.debug_view_enabled = get_parameter("debug_view").as_bool();
        config_.imu_acc_x_limit = get_parameter("imu_acc_x_limit").as_double();
        config_.imu_acc_y_limit = get_parameter("imu_acc_y_limit").as_double();
        config_.imu_acc_z_limit = get_parameter("imu_acc_z_limit").as_double();

        std::string sensorType;
        if (!this->get_parameter("sensor", sensorType))
        {
            RCLCPP_ERROR(this->get_logger(), "no sensor parameters");
            return false;
        }
        // sensorType = this->get_parameter("sensor", sensorType);

        if (sensorType == "velodyne")
        {
            config_.sensor = SensorType::VELODYNE;
            std::cerr << "sensor = VELODYNE" << std::endl;
        }
        else if (sensorType == "ouster")
        {
            config_.sensor = SensorType::OUSTER;
            std::cerr << "sensor=OUSTER" << std::endl;
        }
        else if (sensorType == "livox")
        {
            config_.sensor = SensorType::LIVOX;
            std::cerr << "sensor=LIVOX" << std::endl;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                    "Invalid sensor type: %s. (must be either 'velodyne' or 'ouster')", sensorType.c_str());
            rclcpp::shutdown();
        }

        return true;
    }

    template <typename PointT>
    void featureExtraction::removeClosestFarestPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                                          pcl::PointCloud<PointT> &cloud_out, float min_range,
                                                          float max_range)
    {

        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }
        size_t j = 0;

        PointType point;
        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            //In the bounding box filter
            float pointDis = sqrt(cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y);
            if ((cloud_in.points[i].x > config_.box_size.blindBack && cloud_in.points[i].x < config_.box_size.blindFront &&
                cloud_in.points[i].y > config_.box_size.blindRight && cloud_in.points[i].y < config_.box_size.blindLeft) ||
                (cloud_in.points[i].z > config_.box_size.blindDiskLow && cloud_in.points[i].z < config_.box_size.blindDiskHigh &&
                pointDis < config_.box_size.blindDiskRadius))
            {
                continue;
            }

            // in the range filter
            point.x = cloud_in.points[i].x;
            point.y = cloud_in.points[i].y;
            point.z = cloud_in.points[i].z;

            const Eigen::Vector3f &range = point.getVector3fMap();
            if (std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z))
            {
                if (range.norm() > config_.min_range && range.norm() < config_.max_range)
                {

                    cloud_out.points[j] = cloud_in.points[i];
                    j++;
                }
            }
        }
        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    template <typename Meas>
    bool featureExtraction::synchronize_measurements(MapRingBuffer<Meas> &measureBuf,
                                                     MapRingBuffer<pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr> &lidarBuf)
    {

            if (lidarBuf.getSize() == 0 or measureBuf.getSize() == 0)
                return false;

            double lidar_start_time;
            lidarBuf.getFirstTime(lidar_start_time);

            pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;
            lidarBuf.getFirstMeas(lidar_msg);

            double lidar_end_time = lidar_start_time + lidar_msg->back().time;
            //TODO: Each sensor has diffierent LIDAR_MESSAGE_TIME
            //double lidar_end_time = lidar_start_time +  LIDAR_MESSAGE_TIME;
            //RCLCPP_WARN(this->get_logger(), "LIDAR_MESSAGE_TIME: %f, lidar_start_time: %f ", LIDAR_MESSAGE_TIME, lidar_start_time);

            // obtain the current imu message
            double meas_start_time=0;
            measureBuf.getFirstTime(meas_start_time);

            double meas_end_time=0;
            measureBuf.getLastTime(meas_end_time);

            if (meas_end_time <= lidar_end_time)
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "meas_end_time < lidar_end_time ||"
                                " message order is not perfect! please restart velodyne and imu driver!");
                RCLCPP_WARN(this->get_logger(), "meas_end_time %f <  %f lidar_end_time", meas_end_time, lidar_end_time);
                RCLCPP_WARN(this->get_logger(), "All the lidar data is more recent than all the imu data. Will throw away lidar frame");

                // RCLCPP_WARN(this->get_logger(), "The above messeage should only appear in the first 10 seconds).");
                return false;
            }

            if (meas_start_time >= lidar_start_time)          
            {
                RCLCPP_WARN(this->get_logger(), "throw laser scan, only should happen at the beginning");
                lidarBuf.clean(lidar_start_time);
                RCLCPP_WARN(this->get_logger(), "removed the lidarBuf size % d, measureBuf size % d ", lidarBuf.getSize(), measureBuf.getSize());
                RCLCPP_WARN(this->get_logger(), "meas_start_time: %f > lidar_start_time: %f ", meas_start_time, lidar_start_time);
                return false;
            }
            else
            {
                //RCLCPP_INFO(this->get_logger(),"\033[1;32m----> Both IMU/VIO, laserscan are synchronized!.\033[0m");
                // RCLCPP_WARN(this->get_logger(), "SUCCESS!! meas_first_time: %f <lidar_end_time: %f ", meas_start_time, lidar_end_time);
                return true;
            }
        
    }

    void featureExtraction::imuRemovePointDistortion(double lidar_start_time, double lidar_end_time,
                                                     MapRingBuffer<Imu::Ptr> &imuBuf,
                                                     pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg)
    {

        TicToc t_whole;
        TicToc t_prepare;

        auto &laserCloudIn = *lidar_msg;
        Eigen::Quaterniond q_w_original;
        {
            double t_b_i = lidar_start_time;

            auto after_ptr = imuBuf.measMap_.upper_bound(t_b_i);
            if(after_ptr->first < 0.0001) {
                after_ptr = imuBuf.measMap_.begin();
            }
            if (after_ptr == imuBuf.measMap_.begin())
            {
                q_w_original = after_ptr->second->q_w_i;

                // RCLCPP_INFO_STREAM(this->get_logger(), "find the first imu frame: " << std::to_string(after_ptr->second->time));
            }
            else
            {   
              

                auto before_ptr = after_ptr;
                before_ptr--;
                // RCLCPP_INFO_STREAM(this->get_logger(), "start a: " << std::to_string(t_b_i - before_ptr->second->time));
                // RCLCPP_INFO_STREAM(this->get_logger(), "start b: " << std::to_string(after_ptr->second->time - t_b_i));
                // RCLCPP_INFO_STREAM(this->get_logger(), "start c: " << std::to_string(after_ptr->second->time - before_ptr->second->time));

                double ratio_bi = (t_b_i - before_ptr->second->time) /
                                  (after_ptr->second->time - before_ptr->second->time);

                Eigen::Quaterniond q_w_i_before = before_ptr->second->q_w_i;
                Eigen::Quaterniond q_w_i_after = after_ptr->second->q_w_i;

                q_w_original = q_w_i_before.slerp(ratio_bi, q_w_i_after);
            }
        }

        // RCLCPP_INFO(this->get_logger(), "q_w_original");
        q_w_original_l = q_w_original * T_i_l.rot;
        q_w_original_l.normalized();
        t_w_original_l.x() = 0.0;
        t_w_original_l.y() = 0.0;
        t_w_original_l.z() = 0.0;

        Eigen::Quaterniond q_w_end;
        {
            double t_b_i = lidar_end_time;

            // RCLCPP_INFO(this->get_logger(), "t_b_i: %f", t_b_i);

            auto after_ptr = imuBuf.measMap_.upper_bound(t_b_i);
            if(after_ptr->first < 0.0001) {
                after_ptr = imuBuf.measMap_.begin();
            }
            if (after_ptr == imuBuf.measMap_.begin())
            {
                q_w_end = after_ptr->second->q_w_i;
                // RCLCPP_INFO_STREAM(this->get_logger(), "c2: " << std::to_string(after_ptr->second->time));
            }
            else
            {
                auto before_ptr = after_ptr;
                before_ptr--;

                // RCLCPP_INFO_STREAM(this->get_logger(), "end a: " << std::to_string(t_b_i - before_ptr->second->time));
                // RCLCPP_INFO_STREAM(this->get_logger(), "end b: " << std::to_string(after_ptr->second->time - t_b_i));
                // RCLCPP_INFO_STREAM(this->get_logger(), "end c: " << std::to_string(after_ptr->second->time - before_ptr->second->time));

                double ratio_bi = (t_b_i - before_ptr->second->time) /
                                  (after_ptr->second->time - before_ptr->second->time);
                Eigen::Quaterniond q_w_i_before = before_ptr->second->q_w_i;
                Eigen::Quaterniond q_w_i_after = after_ptr->second->q_w_i;

                q_w_end = q_w_i_before.slerp(ratio_bi, q_w_i_after);
            }
        }

        Eigen::Quaterniond q_original_end = q_w_original.inverse() * q_w_end;

        q_original_end.normalized();

        for (auto &point : laserCloudIn)
        {
            double t_b_i = point.time + lidar_start_time;
            Eigen::Quaterniond q_w_i;

            auto after_ptr = imuBuf.measMap_.lower_bound(t_b_i); 

            if(after_ptr->first < 0.0001) {
                after_ptr = imuBuf.measMap_.begin();
            }
            if (after_ptr == imuBuf.measMap_.begin())
            {
                q_w_i = after_ptr->second->q_w_i;
            }
            else
            {
                auto before_ptr = after_ptr--;
                double ratio_bi = (t_b_i - before_ptr->second->time) /
                                  (after_ptr->second->time - before_ptr->second->time);

                Eigen::Quaterniond q_w_i_before = before_ptr->second->q_w_i;
                Eigen::Quaterniond q_w_i_after = after_ptr->second->q_w_i;

                q_w_i = q_w_i_before.slerp(ratio_bi, q_w_i_after);
            }

            Eigen::Quaterniond q_original_i = q_w_original.inverse() * q_w_i;
            Transformd T_original_i(q_original_i, Eigen::Vector3d::Zero());

            // Transformd T_l_i(imu_Init->imu_laser_R_Gravity.inverse(), T_i_l.trans);
            // Transformd T_l_i(imu_Init->imu_laser_R_Gravity.inverse(), T_i_l.trans);      

            Transformd T_original_i_l = T_l_i * T_original_i * T_i_l;

            if (std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z))
            {
                Eigen::Vector3d pt{point.x, point.y, point.z};

                //TODO: [debug] should correct the following line
                pt = T_original_i_l * pt;
                point.x = pt.x();
                point.y = pt.y();
                point.z = pt.z();
            }
        }
    }

    void featureExtraction::convert_pointcloud_format(pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn,
                                                      pcl::PointCloud<LidarKeypointExtractor::Point>::Ptr &cloud_out)
    {
        LidarKeypointExtractor::Point point;
        for (const auto pt : laserCloudIn)
        {
            if (!std::isfinite(pt.x) ||
                !std::isfinite(pt.y) ||
                !std::isfinite(pt.z))
            {

                continue;
            }
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;
            point.intensity = pt.intensity;
            point.time = pt.time;
            point.laserId = pt.ring;
            cloud_out->push_back(point);
        }
    }

    void featureExtraction::convert_velodyne_scan_order(pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn)
    {

        PointType point;
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(config_.N_SCANS);
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudTobeRegistered(new pcl::PointCloud<pcl::PointXYZHSV>());

        pcl::PointXYZHSV feature_point;
        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());

        for (const auto &pt : laserCloudIn)
        {
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;

            int scanID = pt.ring;
            //float relTime = pt.time;
            // point.intensity = scanID + scanPeriod * relTime;
            point.intensity = pt.intensity;
            laserCloudScans[scanID].push_back(point);
        }

        std::vector<int> scanStartInd(config_.N_SCANS, 0);
        std::vector<int> scanEndInd(config_.N_SCANS, 0);

        laserCloud_ringorder.reset(new pcl::PointCloud<PointType>());

        for (int i = 0; i < config_.N_SCANS; i++)
        {
            scanStartInd[i] = laserCloud->size() + 5;
            *laserCloud += laserCloudScans[i];
            scanEndInd[i] = laserCloud->size() - 6;
        }

        laserCloud_ringorder = laserCloud;

#if 0
        int i=0;
        for (const auto pt :*laserCloud)
        {
            i++;
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;
            point.intensity=i;
           laserCloud_ringorder->push_back(point);
        }
#endif
    }
    

    void featureExtraction::feature_extraction(double lidar_start_time,
                                               pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn)
    {

        TicToc t_whole;
        
        pcl::PointCloud<LidarKeypointExtractor::Point>::Ptr pc(new pcl::PointCloud<LidarKeypointExtractor::Point>);
        //removeClosestFarestPointCloud(laserCloudIn, laserCloudIn, config_.min_range, config_.max_range);
        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
        
        if(laserCloudIn.size()<=10) {
            RCLCPP_WARN(this->get_logger(), "laserCloudIn size is too small: %zu",laserCloudIn.size());
            return;
        }
           
        
        convert_pointcloud_format(laserCloudIn, pc);
        
        

        convert_velodyne_scan_order(laserCloudIn);
        

        KeyPointsExtractor->ComputeKeyPoints(pc, config_.N_SCANS, config_.use_dynamic_mask);

        pcl::PointCloud<PointType>::Ptr edgePoints(new pcl::PointCloud<PointType>());
        edgePoints->reserve(KeyPointsExtractor->EdgesPoints->size());
        pcl::PointCloud<PointType>::Ptr plannerPoints(new pcl::PointCloud<PointType>());
        plannerPoints->reserve(KeyPointsExtractor->PlanarsPoints->size());
        pcl::PointCloud<PointType>::Ptr bobPoints(new pcl::PointCloud<PointType>());
        bobPoints->reserve(KeyPointsExtractor->BlobsPoints->size());
 
        

        auto convertKeypoints = [this](pcl::PointCloud<LidarKeypointExtractor::Point>::Ptr keyPoints, pcl::PointCloud<PointType>::Ptr &feature_points) {
            for (const auto pt : *keyPoints)
            {
                PointType point;
                if (!std::isfinite(pt.x) ||
                    !std::isfinite(pt.y) ||
                    !std::isfinite(pt.z))
                {

                    continue;
                }
                point.x = pt.x;
                point.y = pt.y;
                point.z = pt.z;
                //TODO: [debug]should correct the following intensity line
                point.intensity = pt.time;
                feature_points->push_back(point);
            }
        };

        convertKeypoints(KeyPointsExtractor->EdgesPoints, edgePoints);
        convertKeypoints(KeyPointsExtractor->PlanarsPoints, plannerPoints);
        // convertKeypoints(KeyPointsExtractor->PlanarsPoints, bobPoints);


        //extract features on up and down depth images
        pcl::PointCloud<pcl::PointXYZI>::Ptr depth_up_surf(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr depth_down_surf(new pcl::PointCloud<pcl::PointXYZI>());

        if(LASER_UP_DEPTH_SYNC_SCCUESS== true&& config_.use_up_realsense_points==true) {

            // step 1.1 process depth up points
            if(!all_cloud_buf[0].empty())
            {
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr depthUpPoints(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*all_cloud_buf[0].front(), *depthUpPoints);
            
            tf2::Quaternion up_rotation;
            up_rotation.setRPY(up_realsense_roll* M_PI / 180, up_realsense_pitch * M_PI / 180, 
                               up_realsense_yaw * M_PI / 180);
            tf2::Transform up_transform; 
            up_transform.setOrigin(tf2::Vector3(up_realsense_x,up_realsense_y, up_realsense_z));
            up_transform.setRotation(up_rotation);
            
            std::vector<int> indice1;
            pcl::removeNaNFromPointCloud(*depthUpPoints,*depthUpPoints,indice1);

            //TODO: need to double check the coordinates
            //step1.2: coordinate transform for up points
            tf2::Vector3 point;
            for (int i = 0; i < (int) depthUpPoints->points.size(); i++) {
             
                if (!std::isfinite(depthUpPoints->points[i].x) ||
                    !std::isfinite(depthUpPoints->points[i].y) ||
                    !std::isfinite(depthUpPoints->points[i].z))
                {

                    continue;
                }
                
                point.setX(-depthUpPoints->points[i].y);
                point.setY(depthUpPoints->points[i].x);
                point.setZ(depthUpPoints->points[i].z);
                point=up_transform*point;
                depthUpPoints->points[i].x = point.x();
                depthUpPoints->points[i].y = point.y();
                depthUpPoints->points[i].z = point.z();
            }
            
            RCLCPP_WARN(this->get_logger(), "depthUpPoints: %zu , depth_up_surf: %zu",depthUpPoints->size(),depth_up_surf->size());
            //step1.3: extract features
            DepthFeatureExtraction.uniformfeatureExtraction(depthUpPoints,depth_up_surf,config_.skip_realsense_points);
            }
        } 

        if(LASER_DOWN_DEPTH_SYNC_SCCUESS==true&& config_.use_down_realsense_points==true)
        {
            // step 1.4 process depth down points
           if(!all_cloud_buf[1].empty())
           {
           
            pcl::PointCloud<pcl::PointXYZ>::Ptr depthDownPoints(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*all_cloud_buf[1].front(), *depthDownPoints);
            std::vector<int> indice2;
            pcl::removeNaNFromPointCloud(*depthDownPoints,*depthDownPoints,indice2);
            
            tf2::Quaternion down_rotation;
            down_rotation.setRPY(down_realsense_roll* M_PI / 180, down_realsense_pitch * M_PI / 180, 
                               down_realsense_yaw* M_PI / 180);
 
            tf2::Transform down_transform; 
            down_transform.setOrigin(tf2::Vector3(down_realsense_x,down_realsense_y, down_realsense_z));
            down_transform.setRotation(down_rotation); 
           
            //step1.5: coordinate transform
            tf2::Vector3 down_point;
            for (int i = 0; i < (int) depthDownPoints->points.size(); i++) {

                if (!std::isfinite(depthDownPoints->points[i].x) ||
                    !std::isfinite(depthDownPoints->points[i].y) ||
                    !std::isfinite(depthDownPoints->points[i].z))
                {

                    continue;
                }
                
                down_point.setX(-depthDownPoints->points[i].y);
                down_point.setY(-depthDownPoints->points[i].x);
                down_point.setZ(-depthDownPoints->points[i].z);
                down_point=down_transform*down_point;
                depthDownPoints->points[i].x = down_point.x();
                depthDownPoints->points[i].y = down_point.y();
                depthDownPoints->points[i].z = down_point.z();
            }
             
            //RCLCPP_WARN(this->get_logger(), "depthDownPoints: %d , depth_down_surf: %d",depthDownPoints->size(),depth_down_surf->size()); 
         
            DepthFeatureExtraction.uniformfeatureExtraction(depthDownPoints,depth_down_surf,config_.skip_realsense_points);

           }
         
        } 


        *bobPoints=*depth_down_surf+*depth_up_surf; 
            
        while (! all_cloud_buf[0].empty())
        {
             all_cloud_buf[0].pop();
        }

        while (!all_cloud_buf[1].empty())
        {
             all_cloud_buf[1].pop();
        }


        // {
        //     point.x = q_w_original_l.x();
        //     point.y = q_w_original_l.y();
        //     point.z = q_w_original_l.z();
        //     point.data[3] = q_w_original_l.w();
        //     //    point.data[0]=t_w_original_l.x();
        //     //   point.data[1]=t_w_original_l.y();
        //     //   point.data[2]=t_w_original_l.z();
        //     bobPoints->push_back(point);
        //     laserCloud->push_back(point);
        // }

#if 1  
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr undistortedPoints;
        undistortedPoints = std::make_shared<pcl::PointCloud<point_os::PointcloudXYZITR>>(laserCloudIn);
        //wrap feature message in one message
        publishTopic(lidar_start_time, undistortedPoints, edgePoints, plannerPoints, bobPoints, q_w_original_l);

        // double roll, pitch, yaw;
        // tf2::Quaternion orientation_curr(q_w_original_l.x(), q_w_original_l.y(), q_w_original_l.z(), q_w_original_l.w());
        // tf2::Matrix3x3(orientation_curr).getRPY(roll, pitch, yaw);
        // RCLCPP_INFO(this->get_logger(), "FE Start roll, pitch, yaw %f, %f, %f", roll, pitch, yaw);

        // sensor_msgs::msg::PointCloud2 laserCloudOutMsg;
        // pcl::toROSMsg(*bobPoints, laserCloudOutMsg);
        // laserCloudOutMsg.header.stamp = rclcpp::Time(lidar_start_time*1e9);
        // laserCloudOutMsg.header.frame_id = "sensor_init";
        // pubBobPoints->publish(laserCloudOutMsg);

#else

        sensor_msgs::msg::PointCloud2 cornerPointsLessSharpMsg;
        pcl::toROSMsg(*cornerPointsLessSharp, cornerPointsLessSharpMsg);
        cornerPointsLessSharpMsg.header.stamp = rclcpp::Time(lidar_start_time*1e9);
        cornerPointsLessSharpMsg.header.frame_id = "sensor_init";
        pubCornerPointsLessSharp->publish(cornerPointsLessSharpMsg);

        sensor_msgs::msg::PointCloud2 surfPointsLessFlat2;
        pcl::toROSMsg(*surfPointsLessFlat, surfPointsLessFlat2);
        surfPointsLessFlat2.header.stamp = rclcpp::Time(lidar_start_time*1e9);
        surfPointsLessFlat2.header.frame_id = "sensor_init";
        pubSurfPointsLessFlat->publish(surfPointsLessFlat2);
#endif

    //    printf("scan registration time %f ms *************\n", t_whole.toc());
        if (t_whole.toc() > 100)
            RCLCPP_WARN(this->get_logger(), "scan registration process over 100ms");

        lidarBuf.clean(lidar_start_time);
    }
    
    template <typename Point>
    sensor_msgs::msg::PointCloud2
    featureExtraction::publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, typename pcl::PointCloud<Point>::Ptr thisCloud,
                                    rclcpp::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::msg::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        // if (thisPub->get_subscription_count() != 0)
        //     thisPub->publish(tempCloud);
        return tempCloud;
    }

    void featureExtraction::vioRemovePointDistortion(double lidar_start_time, double lidar_end_time,
                                                     MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> &vioBuf,
                                                     pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg)
    {

        auto &laserCloudIn = *lidar_msg;

        Eigen::Quaterniond q_w_original;
        Eigen::Vector3d t_w_orignal;
        {

            double t_b_i = lidar_start_time;

            auto after_ptr = vioBuf.measMap_.upper_bound(t_b_i);
            if (after_ptr->first < 0.0001)
            {
                after_ptr = vioBuf.measMap_.begin();
            }

            if (after_ptr == vioBuf.measMap_.begin())
            {

                q_w_original.x() = after_ptr->second->pose.pose.orientation.x;
                q_w_original.y() = after_ptr->second->pose.pose.orientation.y;
                q_w_original.z() = after_ptr->second->pose.pose.orientation.z;
                q_w_original.w() = after_ptr->second->pose.pose.orientation.w;

                t_w_orignal.x() = after_ptr->second->pose.pose.position.x;
                t_w_orignal.y() = after_ptr->second->pose.pose.position.y;
                t_w_orignal.z() = after_ptr->second->pose.pose.position.z;

                // RCLCPP_INFO_STREAM(this->get_logger(), "find the first odom frame: " << std::to_string(secs(after_ptr->second)));
            }
            else
            {

                auto before_ptr = after_ptr;
                before_ptr--;

                //    RCLCPP_INFO_STREAM(this->get_logger(), "a: " << std::to_string(t_b_i -
                //    before_ptr->second->time)); RCLCPP_INFO_STREAM(this->get_logger(), "b: " <<
                //    std::to_string(after_ptr->second->time - t_b_i)); RCLCPP_INFO_STREAM(this->get_logger(), "c:
                //    " << std::to_string(after_ptr->second->time -
                //                                            before_ptr->second->time));

                double ratio_bi = (t_b_i - before_ptr->first) /
                                  (after_ptr->first - before_ptr->first);

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

                q_w_original =
                    q_w_i_before.slerp(ratio_bi, q_w_i_after);

                q_w_original.normalized();

                Eigen::Vector3d t_w_i_before;
                t_w_i_before.x() = before_ptr->second->pose.pose.position.x;
                t_w_i_before.y() = before_ptr->second->pose.pose.position.y;
                t_w_i_before.z() = before_ptr->second->pose.pose.position.z;

                Eigen::Vector3d t_w_i_after;
                t_w_i_after.x() = after_ptr->second->pose.pose.position.x;
                t_w_i_after.y() = after_ptr->second->pose.pose.position.y;
                t_w_i_after.z() = after_ptr->second->pose.pose.position.z;

                t_w_orignal =
                    (1 - ratio_bi) * t_w_i_before + ratio_bi * t_w_i_after;
            }
        }

        Transformd T_w_original_i(q_w_original, t_w_orignal);
#if 0
        Transformd T_w_original_l = T_w_original_i * T_i_l;
        q_w_original_l = T_w_original_l.rot;
        t_w_original_l = T_w_original_l.pos;
        q_w_original_l.normalized();
#else
        Transformd T_w_original_l = T_w_original_i;
        q_w_original_l = T_w_original_l.rot;
        q_w_original_l.normalized();
        
        t_w_original_l = T_w_original_l.pos;

#endif

        for (auto &point : laserCloudIn)
        {
            double t_b_i = point.time + lidar_start_time;

            auto after_ptr = vioBuf.measMap_.lower_bound(t_b_i);

            Eigen::Quaterniond q_w_i;
            Eigen::Vector3d t_w_i;

            if (after_ptr->first < 0.0001)
            {
                after_ptr = vioBuf.measMap_.begin();
            }

            if (after_ptr == vioBuf.measMap_.begin())
            {

                q_w_i.x() = after_ptr->second->pose.pose.orientation.x;
                q_w_i.y() = after_ptr->second->pose.pose.orientation.y;
                q_w_i.z() = after_ptr->second->pose.pose.orientation.z;
                q_w_i.w() = after_ptr->second->pose.pose.orientation.w;

                t_w_i.x() = after_ptr->second->pose.pose.position.x;
                t_w_i.y() = after_ptr->second->pose.pose.position.y;
                t_w_i.z() = after_ptr->second->pose.pose.position.z;
            }
            else
            {
                auto before_ptr = after_ptr--;

                double ratio_bi = (t_b_i - before_ptr->first) /
                                  (after_ptr->first - before_ptr->first);

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

                q_w_i =
                    q_w_i_before.slerp(ratio_bi, q_w_i_after);

                q_w_i.normalized();

                Eigen::Vector3d t_w_i_before;
                t_w_i_before.x() = before_ptr->second->pose.pose.position.x;
                t_w_i_before.y() = before_ptr->second->pose.pose.position.y;
                t_w_i_before.z() = before_ptr->second->pose.pose.position.z;

                Eigen::Vector3d t_w_i_after;
                t_w_i_after.x() = after_ptr->second->pose.pose.position.x;
                t_w_i_after.y() = after_ptr->second->pose.pose.position.y;
                t_w_i_after.z() = after_ptr->second->pose.pose.position.z;

                t_w_i =
                    (1 - ratio_bi) * t_w_i_before + ratio_bi * t_w_i_after;
            }

            Transformd T_w_i(q_w_i, t_w_i);

            Transformd T_original_i = T_w_original_i.inverse() * T_w_i;
#if 0
                Transformd T_original_i_l = T_l_i * T_original_i * T_i_l;
#else
            Transformd T_original_i_l = T_original_i;
#endif

            if (std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z))
            {
                Eigen::Vector3d pt{point.x, point.y, point.z};
                pt = T_original_i_l * pt;

                point.x = pt.x();
                point.y = pt.y();
                point.z = pt.z();
            }
        }
    }


    bool featureExtraction::synchronizeLidarDepthMeasurement(double lidar_start_time, std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_in, DepthType depthtype)
    {

        // only use the all_cloud_buf[0] timestamp
        while (!cloud_in.empty()) {
            
            if (cloud_in.front()->header.stamp.sec + cloud_in.front()->header.stamp.nanosec * 1e-9 < lidar_start_time)
            {
                cloud_in.pop();

            }else
            {
                break;
            }
            
        }
        
        if(cloud_in.empty())
        {  
            
         RCLCPP_WARN(this->get_logger(), " SYNC false: cloud_in size : %zu ", cloud_in.size());
         return false;       
        
        }

        double depth_time = cloud_in.front()->header.stamp.sec + cloud_in.front()->header.stamp.nanosec*1e-9;
     
        if (fabs(depth_time-lidar_start_time>1.0))
        {  
            if(depthtype==DepthType::UP_DEPTH)
            RCLCPP_WARN(this->get_logger(), "\033[1;32m lidar and depth camera are not synced | up depth camera time offset %f \033[0m"
                    ,depth_time-lidar_start_time);
            if(depthtype==DepthType::DOWN_DEPTH)
            RCLCPP_WARN(this->get_logger(), "\033[1;32m lidar and depth camera are not synced | down depth camera time offset %f \033[0m"
                    ,depth_time-lidar_start_time);
            return false;

        } else
        {
            RCLCPP_WARN(this->get_logger(), "\033[1;32m----> Depth camera and velodyne are synced\033[0m | cloud_in size : %zu ",
                     cloud_in.size());

            return true;
        }

    }
    
    void featureExtraction::publishTopic(double lidar_start_time, 
                                         pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr laser_no_distortion_points,
                                         pcl::PointCloud<PointType>::Ptr edgePoints,
                                         pcl::PointCloud<PointType>::Ptr plannerPoints, 
                                         pcl::PointCloud<PointType>::Ptr depthPoints,
                                         Eigen::Quaterniond q_w_original_l)
    {
        FeatureHeader.frame_id = WORLD_FRAME;
        FeatureHeader.stamp = rclcpp::Time(lidar_start_time*1e9);
        laserFeature.header = FeatureHeader;
        laserFeature.imu_available = false;
        laserFeature.odom_available = false;
      
        laserFeature.cloud_nodistortion = publishCloud<point_os::PointcloudXYZITR>(pubLaserCloud, laser_no_distortion_points, FeatureHeader.stamp, SENSOR_FRAME);
        laserFeature.cloud_corner = publishCloud<PointType>(pubEdgePoints, edgePoints, FeatureHeader.stamp, SENSOR_FRAME);
        laserFeature.cloud_surface = publishCloud<PointType>(pubPlannerPoints, plannerPoints, FeatureHeader.stamp, SENSOR_FRAME);
        laserFeature.cloud_realsense=publishCloud<PointType>(pubBobPoints, depthPoints, FeatureHeader.stamp, SENSOR_FRAME);
       
        laserFeature.initial_quaternion_x = q_w_original_l.x();
        laserFeature.initial_quaternion_y = q_w_original_l.y();
        laserFeature.initial_quaternion_z = q_w_original_l.z();
        laserFeature.initial_quaternion_w= q_w_original_l.w();
        laserFeature.imu_available = true;
        laserFeature.sensor = 0; // Velodyne
        pubLaserFeatureInfo->publish(laserFeature);
    }

    void featureExtraction::undistortionAndscanregistration()
    {
        LASER_IMU_SYNC_SCCUESS = synchronize_measurements<Imu::Ptr>(imuBuf, lidarBuf);

        LASER_CAMERA_SYNC_SUCCESS = synchronize_measurements<nav_msgs::msg::Odometry::SharedPtr>(visualOdomBuf, lidarBuf);

        if (frameCount > 100 and LASER_CAMERA_SYNC_SUCCESS == true)
            LASER_CAMERA_SYNC_SUCCESS = true;
        else
            LASER_CAMERA_SYNC_SUCCESS = false;


        if ( (LASER_IMU_SYNC_SCCUESS == true or LASER_CAMERA_SYNC_SUCCESS == true) and lidarBuf.getSize() > 0)
        {
            // RCLCPP_INFO(this->get_logger(), "remove distortion");
            double lidar_start_time;
            lidarBuf.getFirstTime(lidar_start_time);
            pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;
            lidarBuf.getFirstMeas(lidar_msg);

            //TODO: [debug] should correct the following line on lidar message time
            double lidar_end_time = lidar_start_time + lidar_msg->back().time;
            //double lidar_end_time = lidar_start_time +  LIDAR_MESSAGE_TIME;

            if(!all_cloud_buf[0].empty())
            {  
                depthtype=DepthType::UP_DEPTH;
                LASER_UP_DEPTH_SYNC_SCCUESS=synchronizeLidarDepthMeasurement(lidar_start_time,all_cloud_buf[0],depthtype);
            }
            if(!all_cloud_buf[1].empty())
            {   
                depthtype=DepthType::DOWN_DEPTH;
                LASER_DOWN_DEPTH_SYNC_SCCUESS=synchronizeLidarDepthMeasurement(lidar_start_time,all_cloud_buf[1],depthtype);    
            }

            if (LASER_IMU_SYNC_SCCUESS == true and LASER_CAMERA_SYNC_SUCCESS == true )
            {
                RCLCPP_INFO(this->get_logger(), "\033[1;32m----> Both IMU ,VIO laserscan are synchronized!.\033[0m");

                vioRemovePointDistortion(lidar_start_time, lidar_end_time, visualOdomBuf, lidar_msg);
            }

            if (LASER_IMU_SYNC_SCCUESS == false and LASER_CAMERA_SYNC_SUCCESS == true  )
            {
               // RCLCPP_INFO(this->get_logger(), "\033[1;32m----> VIO and laserscan are synchronized!.\033[0m");
                
                vioRemovePointDistortion(lidar_start_time, lidar_end_time, visualOdomBuf, lidar_msg);
            }

            if (LASER_IMU_SYNC_SCCUESS == true and LASER_CAMERA_SYNC_SUCCESS == false)
            {
               // RCLCPP_INFO(this->get_logger(), "\033[1;32m----> IMU and laserscan are synchronized!.\033[0m");
                //if(delay_count_>60)
                imuRemovePointDistortion(lidar_start_time, lidar_end_time, imuBuf, lidar_msg);
            }

            //TODO: Clean the realsense feature extraction code 
            if(config_.sensor==SensorType::LIVOX)
            {
                pcl::PointCloud<PointType>::Ptr plannerPoints(new pcl::PointCloud<PointType>());
                plannerPoints->reserve(lidar_msg->points.size());
                pcl::PointCloud<PointType>::Ptr edgePoints(new pcl::PointCloud<PointType>());
                edgePoints->reserve(lidar_msg->points.size());
                pcl::PointCloud<PointType>::Ptr bobPoints(new pcl::PointCloud<PointType>());
                bobPoints->reserve(lidar_msg->points.size());

                DepthFeatureExtraction.uniformfeatureExtraction(lidar_msg, plannerPoints, config_.skip_realsense_points,config_.min_range);
              
                assert (plannerPoints->size()>0);     
                publishTopic(lidar_start_time, lidar_msg, edgePoints, plannerPoints, bobPoints, q_w_original_l);   

            }else
            {
                feature_extraction(lidar_start_time, *lidar_msg);
            }

            LASER_CAMERA_SYNC_SUCCESS = false;
            LASER_IMU_SYNC_SCCUESS = false;
            LASER_UP_DEPTH_SYNC_SCCUESS=false;
            LASER_DOWN_DEPTH_SYNC_SCCUESS=false;

        }else
        {
            RCLCPP_WARN(this->get_logger(), "sync unsuccessfull, skipping scan frame");
        }
    }


    // Transform to z-up and limit the acceleration
    void featureExtraction::normalizeImuData(const sensor_msgs::msg::Imu::SharedPtr imu_in, Imu::Ptr imuData)
    {
        assert(imuData != nullptr);
        assert(IMU_INIT == true);
        double timestamp = imu_in->header.stamp.sec + imu_in->header.stamp.nanosec * 1e-9;
        Eigen::Vector3d accel;
        accel << imu_in->linear_acceleration.x, imu_in->linear_acceleration.y,
            imu_in->linear_acceleration.z;

        Eigen::Vector3d gyr;
        gyr << imu_in->angular_velocity.x, imu_in->angular_velocity.y,
            imu_in->angular_velocity.z;
        
        gyr = imu_Init->imu_laser_R_Gravity * gyr;
        accel = imu_Init->imu_laser_R_Gravity * accel;

        Eigen::Vector3d acc_mean = imu_Init->acc_mean;
        Eigen::Vector3d gyr_mean = imu_Init->gyr_mean;
        acc_mean = imu_Init->imu_laser_R_Gravity * acc_mean;
        gyr_mean = imu_Init->imu_laser_R_Gravity * gyr_mean;

        Eigen::Vector3d acc_diff = accel - acc_mean;
        Eigen::Vector3d gyr_diff = gyr - gyr_mean;

        if (abs(acc_diff(0)) > config_.imu_acc_x_limit){
            RCLCPP_WARN(this->get_logger(), "IMU acc x limit exceeded: %f", acc_diff(0));
            acc_diff(0) = acc_diff(0) > 0 ? config_.imu_acc_x_limit : -config_.imu_acc_x_limit;
        }
        if (abs(acc_diff(1)) > config_.imu_acc_y_limit){
            RCLCPP_WARN(this->get_logger(), "IMU acc y limit exceeded: %f", acc_diff(1));
            acc_diff(1) = acc_diff(1) > 0 ? config_.imu_acc_y_limit : -config_.imu_acc_y_limit;
        }
        if (abs(acc_diff(2)) > config_.imu_acc_z_limit){
            RCLCPP_WARN(this->get_logger(), "IMU acc z limit exceeded: %f", acc_diff(2));
            acc_diff(2) = acc_diff(2) > 0 ? config_.imu_acc_z_limit : -config_.imu_acc_z_limit;
        }
        // Leave gyr limit for now

        double gravity = 9.8105;
        imuData->time = timestamp;
        imuData->acc = acc_diff + acc_mean;
        imuData->acc = imuData->acc * gravity / imu_Init->acc_mean.norm();
        imuData->gyr = gyr_diff + gyr_mean;
    }

    void featureExtraction::imu_Handler(const sensor_msgs::msg::Imu::SharedPtr msg_in)
    {   
        if (config_.lidar_flip) 
        {
            msg_in->linear_acceleration.y *= -1.0;
            msg_in->linear_acceleration.z *= -1.0;
            msg_in->angular_velocity.y *= -1.0;
            msg_in->angular_velocity.z *= -1.0;
        }
       
        m_buf.lock();
        auto msg = msg_in;
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        double lastImuTime = 0.0;
        double dt = m_imuPeriod;
      
        if(imuBuf.getLastTime(lastImuTime))
        {
            dt = timestamp - lastImuTime;
            if(abs(dt - m_imuPeriod) > m_imuPeriod * IMU_TIME_LENIENCY)
            {
                // IMU timestamp jumped - quietly assume a normal delta t
                dt = m_imuPeriod;
            }
        }
        Eigen::Vector3d accel;
        accel << msg->linear_acceleration.x, msg->linear_acceleration.y,
            msg->linear_acceleration.z;
        Eigen::Vector3d gyr;
        gyr << msg->angular_velocity.x, msg->angular_velocity.y,
            msg->angular_velocity.z;

        Imu::Ptr imudata = std::make_shared<Imu>();
        imudata->time = timestamp;
        if(IMU_INIT ==true and config_.sensor==SensorType::LIVOX)
        {  
            // normalizeImuData(msg, imudata);

            //TODO: Gravity should read from yaml file.
            double gravity = 9.8105;

            gyr = imu_Init->imu_laser_R_Gravity * gyr;
            accel = imu_Init->imu_laser_R_Gravity * imudata->acc;

            imudata->acc = accel*gravity/imu_Init->acc_mean.norm();
            imudata->gyr = gyr;
        }else
        {

            imudata->acc = accel;
            imudata->gyr = gyr;
        }
       

        if (!imuBuf.empty())
        {
            const Eigen::Quaterniond rot_last = imuBuf.measMap_.rbegin()->second->q_w_i;
            Eigen::Vector3d gyr_last = imuBuf.measMap_.rbegin()->second->gyr;
            const double &time_last = imuBuf.measMap_.rbegin()->second->time;

            double dt = timestamp - time_last;
            Eigen::Vector3d delta_angle = dt * 0.5 * (gyr + gyr_last);
            Eigen::Quaterniond delta_r =
                Sophus::SO3d::exp(delta_angle).unit_quaternion();

            Eigen::Quaterniond rot = rot_last * delta_r;
            rot.normalized();
            imudata->q_w_i = rot;
            
        }
        else
        {
            // In fact this function should only works for the first pose
            if (config_.use_imu_roll_pitch)
            {
                // Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

                // auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
                // auto rot = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) *
                //            Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                //            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
                double roll, pitch, yaw;

                tf2::Quaternion orientation_curr(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
                tf2::Matrix3x3(orientation_curr).getRPY(roll, pitch, yaw);
                // RCLCPP_INFO(this->get_logger(), "FE Start roll, pitch, yaw %f, %f, %f", roll, pitch, yaw);
                tf2::Quaternion first_orientation;
                tf2::Quaternion yaw_quat;
                yaw_quat.setRPY(0,0,-yaw);
                first_orientation = yaw_quat*orientation_curr;
                // first_orientation = orientation_curr;

                imudata->q_w_i = Eigen::Quaterniond(first_orientation.w(), first_orientation.x(), first_orientation.y(), first_orientation.z());
            }
        }

        imuBuf.addMeas(imudata, timestamp);

        double vio_first_time = 0;
        visualOdomBuf.getFirstTime(vio_first_time);

        double lidar_first_time = 0;
        if(lidarBuf.getFirstTime(lidar_first_time)) {
            // std::cout << std::fixed << std::setprecision(10);
            // std::cout << "imu Timestamp: " << timestamp << std::endl; 
            // std::cout << "lidar_last_time: " << lidar_last_time << std::endl;
            // std::cout << "LIDAR_MESSAGE_TIME: " << LIDAR_MESSAGE_TIME << std::endl;        
            
            if (timestamp > lidar_first_time + LIDAR_MESSAGE_TIME + 0.05)
            {   
              
                if(config_.sensor==SensorType::LIVOX)
                {   
                   
                    double first_time = 0.0;
                    imuBuf.getFirstTime(first_time);

                    if (timestamp-first_time>200*m_imuPeriod and IMU_INIT==false)
                    {   
                        //TODO: IMUInit might be not necessary since it is only for accleration 
                        imu_Init->imuInit(imuBuf);
                        IMU_INIT=true;
                        imuBuf.clean(timestamp);
                        std::cout<<"IMU Initialization Process Finish! "<<std::endl;

                    }
                    if(IMU_INIT==true)
                    {   
                        // RCLCPP_INFO(get_logger(), "In distortion loop");
                       undistortionAndscanregistration();
                       double lidar_first_time;
                       lidarBuf.getFirstTime(lidar_first_time);
                       lidarBuf.clean(lidar_first_time);
                    }
                }else
                {
                   // undistortionAndscanregistration();
                    double lidar_first_time;
                    lidarBuf.getFirstTime(lidar_first_time);
                    lidarBuf.clean(lidar_first_time);
                }
            }
            // else{
            //     RCLCPP_INFO(this->get_logger(), "time diff: %f", timestamp - lidar_first_time);
            // }
        }
       

        m_buf.unlock();
    }

    void featureExtraction::visual_odom_Handler(const nav_msgs::msg::Odometry::SharedPtr visualOdometry)
    {
       // RCLCPP_WARN(this->get_logger(), "visualodomhandler");
        m_buf.lock();
        visualOdomBuf.addMeas(visualOdometry, visualOdometry->header.stamp.sec + visualOdometry->header.stamp.nanosec*1e-9);
        m_buf.unlock();
    }

    void featureExtraction::assignTimeforPointCloud(pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_)
    {
        size_t cloud_size = laserCloudIn_ptr_->size();
        RCLCPP_DEBUG(this->get_logger(), "\n\ninput cloud size: %zu \n",cloud_size);
        pointCloudwithTime.reset(new pcl::PointCloud<point_os::PointcloudXYZITR>());
        pointCloudwithTime->reserve(cloud_size);

        point_os::PointcloudXYZITR point;
        for (size_t i = 0; i < cloud_size; i++)
        {
            point.x = laserCloudIn_ptr_->points[i].x;
            point.y = laserCloudIn_ptr_->points[i].y;
            point.z = laserCloudIn_ptr_->points[i].z;
            point.intensity = laserCloudIn_ptr_->points[i].intensity;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (config_.N_SCANS == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (config_.N_SCANS - 1) || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else if (config_.N_SCANS == 32)
            {
                scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
                if (scanID > (config_.N_SCANS - 1) || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else if (config_.N_SCANS == 64)
            {
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = config_.N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                // use [0 50]  > 50 remove outlies
                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
                // ROS_BREAK();
            }

            point.ring = scanID;
            float rel_time = (columnTime * int(i / config_.N_SCANS) + laserTime * (i % config_.N_SCANS)) / scanPeriod;
            float pointTime = rel_time * scanPeriod;
            point.time = pointTime;
            pointCloudwithTime->push_back(point);
        }
    }

    void featureExtraction::laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {  

        if (imuBuf.empty() || delay_count_++ <= 5)
        {
            RCLCPP_WARN(this->get_logger(), "imu buf empty");
            return;
        }

        frameCount = frameCount + 1;
        if (frameCount % config_.skipFrame != 0)
            return;

        m_buf.lock();

        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud(
            new pcl::PointCloud<point_os::PointcloudXYZITR>());

        tmpOusterCloudIn.reset(new pcl::PointCloud<point_os::OusterPointXYZIRT>());
        if (config_.provide_point_time)
        {

            if (config_.sensor == SensorType::VELODYNE)
            {
                
                pcl::fromROSMsg(*laserCloudMsg, *pointCloud);


            }
            else if (config_.sensor == SensorType::OUSTER)
            {
                // Convert to Velodyne format
                pcl::fromROSMsg(*laserCloudMsg, *tmpOusterCloudIn);
                pointCloud->points.resize(tmpOusterCloudIn->size());
                pointCloud->is_dense = tmpOusterCloudIn->is_dense;
                for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
                {
                    auto &src = tmpOusterCloudIn->points[i];
                    auto &dst = pointCloud->points[i];
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    dst.intensity = src.intensity;
                    dst.ring = src.ring;
                    dst.time = src.t * 1e-9f;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),"Unknown sensor type: %d", int(sensor));
                rclcpp::shutdown();
            }
        }
        else
        {
            pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn_ptr_);
            assignTimeforPointCloud(laserCloudIn_ptr_);
            pointCloud = pointCloudwithTime;

        }

        if (config_.lidar_flip) 
        {
            for (uint i = 1; i < pointCloud->points.size(); i++)
            {
                pointCloud->points[i].y *= -1.0;
                pointCloud->points[i].z *= -1.0;
            }
        }

        // if (lidarBuf.getSize() > 1)
        // {
        //     // Remove oldest lidar data from the buffer
        //     double lidar_first_time;
        //     lidarBuf.getFirstTime(lidar_first_time);
        //     lidarBuf.clean(lidar_first_time);
        // }
        // RCLCPP_WARN(this->get_logger(), "lidar buf size: %d", lidarBuf.getSize());
     
        std::size_t curLidarBufferSize = lidarBuf.getSize();
      
        if(curLidarBufferSize > 2) {
            RCLCPP_WARN(this->get_logger(), "Large lidar buffer size: %zu", curLidarBufferSize);
            RCLCPP_WARN(this->get_logger(), "  IMU and lidar times are likely out of sync");
        }

        while (curLidarBufferSize >= 50)
            {
                double lidar_first_time;
                lidarBuf.getFirstTime(lidar_first_time);
                lidarBuf.clean(lidar_first_time);
            RCLCPP_WARN(this->get_logger(), "Lidar buffer too large, dropping frame");
            curLidarBufferSize = lidarBuf.getSize();
        }
        
        removeClosestFarestPointCloud(*pointCloud, *pointCloud, config_.min_range, config_.max_range);
        lidarBuf.addMeas(pointCloud, laserCloudMsg->header.stamp.sec + laserCloudMsg->header.stamp.nanosec*1e-9);
        
        
        //undistortionAndscanregistration();

        m_buf.unlock();
    }

    Eigen::Matrix3d featureExtraction::getPitchTF(double pitch_degrees)
    {
        const double deg2rad = M_PI / 180.0;
        double theta = pitch_degrees * deg2rad;
        Eigen::Matrix3d R_pitch;
        R_pitch << std::cos(theta), 0, std::sin(theta),
                0, 1, 0,
                -std::sin(theta), 0, std::cos(theta);
        return R_pitch;
    }

    //TODO: add livox mid360 handler
    void featureExtraction::livoxHandler(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
    {     

        if (imuBuf.empty())
        {   
            RCLCPP_WARN(this->get_logger(), "imu buf empty");
            return;
        }
         
        frameCount = frameCount + 1;
        if (frameCount % config_.skipFrame != 0)
            return; 

        if (config_.lidar_flip) 
        {
            for (uint i = 1; i < msg->point_num; i++)
            {
                msg->points[i].y *= -1.0;
                msg->points[i].z *= -1.0;
            }
        }

        m_buf.lock();
        
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud(
            new pcl::PointCloud<point_os::PointcloudXYZITR>());
        
        pointCloud->points.resize(msg->point_num);
        //   Eigen::Matrix3d R_pitch = getPitchTF(config_.livox_pitch);
        Eigen::Matrix3d Roll_Pitch_Gravity_Matrix;
        Roll_Pitch_Gravity_Matrix=imu_Init->imu_laser_R_Gravity;


        if (config_.sensor == SensorType::LIVOX)
        { 
            
           if(config_.provide_point_time)
           {     
               
                for (uint i=1; i <msg->point_num; i++)
                {
                    if ((msg->points[i].line < config_.N_SCANS) &&
                        ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
                    {   
                        Eigen::Vector3d point(msg->points[i].x, msg->points[i].y, msg->points[i].z);
                        Eigen::Vector3d transformed_point = Roll_Pitch_Gravity_Matrix * point; // in case if pitch provided
                        pointCloud->points[i].x = transformed_point.x();
                        pointCloud->points[i].y = transformed_point.y();
                        pointCloud->points[i].z = transformed_point.z();
                        
                        pointCloud->points[i].intensity = msg->points[i].reflectivity;
                        //pointCloud->points[i].time = msg->points[i].offset_time / float(1000000);  // time of each laser points
                        //TODO:Temporary fix the timestamp for livox
                        pointCloud->points[i].time = msg->points[i].offset_time / float(1000000000);  // time of each laser points
                        
                    }
                }
            
         

           }
           else
           {
             
    
             RCLCPP_ERROR(this->get_logger(), "Please check yaml or livox driver to provide the timestamp for each point");
             rclcpp::shutdown();
           }

        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),"Unknown sensor type: %d", int(sensor));
            rclcpp::shutdown();
        }
        
     
        
        std::size_t curLidarBufferSize = lidarBuf.getSize();
       
        if(curLidarBufferSize > 2) {
            RCLCPP_WARN(this->get_logger(), "Large lidar buffer size: %zu", curLidarBufferSize);
            RCLCPP_WARN(this->get_logger(), "  IMU and lidar times are likely out of sync");
        }
        
        
        while (curLidarBufferSize >= 50)
        {
            double lidar_first_time;
            lidarBuf.getFirstTime(lidar_first_time);
            lidarBuf.clean(lidar_first_time);
            RCLCPP_WARN(this->get_logger(), "Lidar buffer too large, dropping frame");
            curLidarBufferSize = lidarBuf.getSize();
        }
        
       
        // sensor_msgs::msg::PointCloud2 tempCloud;
        // tempCloud=publishCloud<point_os::PointcloudXYZITR>(pubBobPoints, pointCloud, msg->header.stamp, SENSOR_FRAME);

        removeClosestFarestPointCloud(*pointCloud, *pointCloud, config_.min_range, config_.max_range);
        lidarBuf.addMeas(pointCloud, msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9);

        m_buf.unlock();
    }


    

} // namespace arise_slam
