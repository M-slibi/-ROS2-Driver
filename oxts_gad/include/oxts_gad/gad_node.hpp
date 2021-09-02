/**
 * \file tests.h
 * Contains anything which is common to all the unit tests.
 */
#ifndef GAD_NODE_HPP
#define GAD_NODE_HPP

#include <memory>
#include <vector>
#include <map>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "oxts/gal-cpp/gad.hpp"
#include "oxts/gal-cpp/gad_handler.hpp"


namespace OxTS
{

/**
 * Convert position data in ROS geometry_msgs/PoseWithCovariance to GAD Position
 * @param msg Pose message to convert from
 * @param ga_out GAD object to store converted data in
 */
void pose_with_covariance_to_gad_position(
  const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg, 
  OxTS::GadPosition& ga_out 
);
/**
 * Convert orientation data in ROS geometry_msgs/PoseWithCovariance to GAD Attitude
 * @param msg Pose message to convert from
 * @param ga_out GAD object to store converted data in
 */
void pose_with_covariance_to_gad_attitude(
  const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg,
  OxTS::GadAttitude& ga_out
);
/**
 * Convert orientation data in ROS geometry_msgs/PoseWithCovariance to GAD Attitude
 * @param msg Pose message to convert from
 * @param ga_out GAD object to store converted data in
 */
void pose_with_covariance_to_gad(
  const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg,
  OxTS::GadPosition& gp_out,
  OxTS::GadAttitude& ga_out
);

/**
 * Convert orientation data in ROS geometry_msgs/PoseWithCovarianceStamped to 
 * GAD Attitude. 
 * @param msg Pose message to convert from
 * @param ga_out GAD object to store converted data in
 */
void pose_with_covariance_stamped_to_gad(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, 
  OxTS::GadPosition& gp_out,
  OxTS::GadAttitude& ga_out 
);

/**
 * Convert velocity data in ROS geometry_msgs/TwistWithCovariance to GAD Velocity
 * @param msg Twist message to convert from
 * @param gv_out GAD object to store converted data in
 */
void twist_with_covariance_to_gad_velocity(
  const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg, 
  OxTS::GadVelocity& gv_out 
);

/**
 * Convert velocity data in ROS geometry_msgs/TwistWithCovarianceStamped to 
 * GAD Velocity
 * @param msg Twist message to convert from
 * @param gv_out GAD object to store converted data in
 */
void twist_with_covariance_stamped_to_gad(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg, 
  OxTS::GadVelocity& gv_out 
);

enum OUTPUT_MODE
{
  UDP = 0,
  CSV = 1
};
/**
 * ROS node to listen for specified ROS topics, convert them to GAD, and either 
 * send them to an OxTS INS or to CSV.
 */
class GadNode : public rclcpp::Node
{
  private:
  // Configurable parameters
  int         output_mode;
  std::string unit_ip;
  std::string file_out;
  std::string nav_sat_fix_topic;
  std::string odom_topic;
  std::map<std::string, int> stream_ids;
  // Other class members
  OxTS::GadHandler gad_handler; 
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subNavSatFix_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;
  // Topic callbacks
  void nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  public:
  // Constructor
  GadNode(const rclcpp::NodeOptions &options)
  : Node("oxts_gad", options)
  {
    // Load parameters
    output_mode       = this->declare_parameter("output_mode", 0);
    unit_ip           = this->declare_parameter("unit_ip", std::string(""));
    file_out          = this->declare_parameter("file_out", std::string(""));
    nav_sat_fix_topic = this->declare_parameter("nav_sat_fix_topic", "/ins/nav_sat_fix");
    odom_topic        = this->declare_parameter("odom_topic", "/ins/odom");
    stream_ids["NAV_SAT_FIX_POS"] = this->declare_parameter("nav_sat_fix_pos_stream_id", 130);
    stream_ids["ODOM_POS"] = this->declare_parameter("odom_pos_stream_id", 140);
    stream_ids["ODOM_VEL"] = this->declare_parameter("odom_vel_stream_id", 141);
    stream_ids["ODOM_ATT"] = this->declare_parameter("odom_att_stream_id", 142);


    switch (output_mode)
    {
      case OUTPUT_MODE::UDP:
        gad_handler.SetEncoderToBin();
        gad_handler.SetOutputModeToUdp(unit_ip);
        RCLCPP_INFO(this->get_logger(), "INS IP: %s", unit_ip.c_str());
        break;
      case OUTPUT_MODE::CSV:
        gad_handler.SetEncoderToCsv();
        gad_handler.SetOutputModeToFile(file_out);
        RCLCPP_INFO(this->get_logger(), "Output file: %s", file_out.c_str());
        break;
      default:
        gad_handler.SetEncoderToBin();
        gad_handler.SetOutputModeToUdp(unit_ip);
        RCLCPP_INFO(this->get_logger(), "INS IP: %s", unit_ip.c_str());
        break;
    }


    subNavSatFix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        nav_sat_fix_topic, 
        10, 
        std::bind(&GadNode::nav_sat_fix_callback, this, std::placeholders::_1)
    );
    subOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 
        10, 
        std::bind(&GadNode::odometry_callback, this, std::placeholders::_1)
    );
  }

};

} // namespace OxTS

#endif // GAD_NODE_HPP