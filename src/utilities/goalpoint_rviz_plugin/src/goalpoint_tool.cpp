#include <goalpoint_tool.hpp>

#include <string>
#include <tf2/LinearMath/Quaternion.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <rviz_common/properties/bool_property.hpp>

namespace goalpoint_rviz_plugin
{
GoalpointTool::GoalpointTool()
: rviz_default_plugins::tools::PoseTool(), qos_profile_(5), use_pose_topic_(true)
{
  shortcut_key_ = 'w';

  topic_property_ = new rviz_common::properties::StringProperty("Topic", "goalpoint", "The topic on which to publish navigation waypoints.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);

  use_pose_property_ = new rviz_common::properties::BoolProperty("Use Pose Topic", true,
                                       "If true, publishes PoseStamped on /goal_pose. If false, publishes PointStamped on /goal_point.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

GoalpointTool::~GoalpointTool() = default;

void GoalpointTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("Goalpoint");
  updateTopic();
  vehicle_z = 0;
}

void GoalpointTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  sub_ = raw_node->template create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5 ,std::bind(&GoalpointTool::odomHandler,this,std::placeholders::_1));

  use_pose_topic_ = use_pose_property_->getBool();

  if (use_pose_topic_) {
    pub_pose_ = raw_node->template create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", qos_profile_);
    pub_.reset();
  } else {
    pub_ = raw_node->template create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", qos_profile_);
    pub_pose_.reset();
  }

  pub_joy_ = raw_node->template create_publisher<sensor_msgs::msg::Joy>("/joy", qos_profile_);
  clock_ = raw_node->get_clock();
}

void GoalpointTool::odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

void GoalpointTool::onPoseSet(double x, double y, double theta)
{
  sensor_msgs::msg::Joy joy;

  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  joy.header.stamp = clock_->now();
  joy.header.frame_id = "goalpoint_tool";
  pub_joy_->publish(joy);

  if (use_pose_topic_) {
    geometry_msgs::msg::PoseStamped goalpose;
    goalpose.header.frame_id = "map";
    goalpose.header.stamp = joy.header.stamp;
    goalpose.pose.position.x = x;
    goalpose.pose.position.y = y;
    goalpose.pose.position.z = vehicle_z;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    goalpose.pose.orientation.x = q.x();
    goalpose.pose.orientation.y = q.y();
    goalpose.pose.orientation.z = q.z();
    goalpose.pose.orientation.w = q.w();

    pub_pose_->publish(goalpose);
    usleep(10000);
    pub_pose_->publish(goalpose);
  } else {
    geometry_msgs::msg::PointStamped goalpoint;
    goalpoint.header.frame_id = "map";
    goalpoint.header.stamp = joy.header.stamp;
    goalpoint.point.x = x;
    goalpoint.point.y = y;
    goalpoint.point.z = vehicle_z;

    pub_->publish(goalpoint);
    // Publish twice for reliability
    usleep(10000);
    pub_->publish(goalpoint);
  }
}
}

#include <pluginlib/class_list_macros.hpp> 
PLUGINLIB_EXPORT_CLASS(goalpoint_rviz_plugin::GoalpointTool, rviz_common::Tool)
