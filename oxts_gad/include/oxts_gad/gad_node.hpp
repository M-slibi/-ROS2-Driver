/**
 * \file tests.h
 * Contains anything which is common to all the unit tests.
 */
#ifndef GAD_NODE_HPP
#define GAD_NODE_HPP

#include <memory>
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "oxts/gal-cpp/gad.hpp"
#include "oxts/gal-cpp/gad_handler.hpp"

#include "oxts_gad/gad_conversions.hpp"

namespace OxTS
{


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
  int         timestamp_mode;
  std::string timestamp;
  double      utc_offset;
  std::string unit_ip;
  std::string file_out;
  std::string nav_sat_fix_topic;
  std::string odom_topic;
  std::string pose_with_cov_stamped_topic;
  std::string twist_with_cov_stamped_topic_vel;
  std::string twist_with_cov_stamped_topic_speed;
  std::map<std::string, int> stream_ids;
  std::string OFF_TOPIC = "/off";
  // Other class members
  OxTS::GadHandler gad_handler; 
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subNavSatFix_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr 
    subPoseWithCovarianceStamped_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr 
    subTwistWithCovarianceStampedVel_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr 
    subTwistWithCovarianceStampedSpeed_;
  // Topic callbacks
  void nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pose_with_cov_stamped_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg
  );
  void twist_with_cov_stamped_vel_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg
  );
  void twist_with_cov_stamped_speed_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg
  );


  public:
  // Constructor
  GadNode(const rclcpp::NodeOptions &options)
  : Node("oxts_gad", options)
  {
    // Load parameters
    output_mode        = this->declare_parameter("output_mode", 0);
    utc_offset         = this->declare_parameter("utc_offset", UTC_OFFSET_DEFAULT);
    timestamp          = this->declare_parameter("timestamp", "gps");
    unit_ip            = this->declare_parameter("unit_ip", std::string(""));
    file_out           = this->declare_parameter("file_out", std::string(""));
    nav_sat_fix_topic  = this->declare_parameter("nav_sat_fix_topic", "/ins/nav_sat_fix");
    stream_ids["NAV_SAT_FIX_POS"] =
      this->declare_parameter("nav_sat_fix_pos_stream_id", 130);
    odom_topic        = this->declare_parameter("odom_topic", "/ins/odom");
    stream_ids["ODOM_POS"] = this->declare_parameter("odom_pos_stream_id", 140);
    stream_ids["ODOM_ATT"] = this->declare_parameter("odom_att_stream_id", 141);
    stream_ids["ODOM_VEL"] = this->declare_parameter("odom_vel_stream_id", 142);
    pose_with_cov_stamped_topic = 
      this->declare_parameter("pose_with_cov_stamped_topic", "/ins/pose");
    stream_ids["POSE_WITH_COV_STAMPED_POS"] = 
      this->declare_parameter("pose_with_cov_stamped_pos_stream_id", 150);
    stream_ids["POSE_WITH_COV_STAMPED_ATT"]  =
      this->declare_parameter("pose_with_cov_stamped_att_stream_id", 151);
    twist_with_cov_stamped_topic_vel = 
      this->declare_parameter("twist_with_cov_stamped_topic_vel", "/ins/twist");
    stream_ids["TWIST_WITH_COV_STAMPED_VEL"] =
      this->declare_parameter("twist_with_cov_stamped_vel_stream_id", 152);
    twist_with_cov_stamped_topic_speed = 
      this->declare_parameter("twist_with_cov_stamped_topic_speed", "/ins/twist");
    stream_ids["TWIST_WITH_COV_STAMPED_SPEED"] =
      this->declare_parameter("twist_with_cov_stamped_speed_stream_id", 153);

    this->timestamp_mode = TIMESTAMP_MODE[this->timestamp];
    RCLCPP_INFO(this->get_logger(), "GAD timestamp mode: %s  (%d)", 
      this->timestamp.c_str(), 
      this->timestamp_mode 
    );


    switch (output_mode)
    {
      case OUTPUT_MODE::UDP:
        gad_handler.SetEncoderToBin();
        gad_handler.SetOutputModeToUdp(unit_ip);
        RCLCPP_INFO(this->get_logger(), "GAD output to IP: %s", unit_ip.c_str());
        break;
      case OUTPUT_MODE::CSV:
        gad_handler.SetEncoderToCsv();
        gad_handler.SetOutputModeToFile(file_out);
        RCLCPP_INFO(this->get_logger(), "GAD output to file: %s", file_out.c_str());
        break;
      default:
        gad_handler.SetEncoderToBin();
        gad_handler.SetOutputModeToUdp(unit_ip);
        RCLCPP_INFO(this->get_logger(), "GAD output to IP: %s", unit_ip.c_str());
        break;
    }

    std::ostringstream oss;
    oss << "\n=================================================="; 
    oss << "\nMSG       : TOPIC -> GAD TYPE [STREAM ID]"; 
    oss << "\n--------------------------------------------------\n"; 

    if (nav_sat_fix_topic != OFF_TOPIC) {
      subNavSatFix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          nav_sat_fix_topic, 
          10, 
          std::bind(&GadNode::nav_sat_fix_callback, this, std::placeholders::_1)
      );
      oss << "NavSatFix : " << nav_sat_fix_topic << " -> GadPosition " << stream_ids["NAV_SAT_FIX_POS"] << "\n";
    }
    if (odom_topic != OFF_TOPIC) {
      subOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
          odom_topic, 
          10, 
          std::bind(&GadNode::odometry_callback, this, std::placeholders::_1)
      );
      oss << "Odometry  : " << odom_topic << " -> GadPosition " << stream_ids["ODOM_POS"] << "\n"
          << "Odometry  : " << odom_topic << " -> GadVelocity " << stream_ids["ODOM_VEL"] << "\n"
          << "Odometry  : " << odom_topic << " -> GadAttitude " << stream_ids["ODOM_ATT"] << "\n";
    }
    if (pose_with_cov_stamped_topic != OFF_TOPIC) {
      subPoseWithCovarianceStamped_ = 
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          pose_with_cov_stamped_topic, 
          10, 
          std::bind(&GadNode::pose_with_cov_stamped_callback, this, std::placeholders::_1)
      );
      oss << "Pose      : " << pose_with_cov_stamped_topic << " -> GadPosition " << stream_ids["POSE_WITH_COV_STAMPED_POS"] << "\n"
          << "Pose      : " << pose_with_cov_stamped_topic << " -> GadAttitude " << stream_ids["POSE_WITH_COV_STAMPED_ATT"] << "\n";
    }
    if (twist_with_cov_stamped_topic_vel != OFF_TOPIC) {
      subTwistWithCovarianceStampedVel_ = 
        this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
          twist_with_cov_stamped_topic_vel, 
          10, 
          std::bind(&GadNode::twist_with_cov_stamped_vel_callback, this, std::placeholders::_1)
      );
      oss << "Twist     : " << twist_with_cov_stamped_topic_vel << " -> GadVelocity " << stream_ids["TWIST_WITH_COV_STAMPED_VEL"] << "\n";
    }
    if (twist_with_cov_stamped_topic_speed != OFF_TOPIC) {
      subTwistWithCovarianceStampedSpeed_ = 
        this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
          twist_with_cov_stamped_topic_speed, 
          10, 
          std::bind(&GadNode::twist_with_cov_stamped_speed_callback, this, std::placeholders::_1)
      );
      oss << "Twist     : " << twist_with_cov_stamped_topic_speed << " -> GadSpeed " << stream_ids["TWIST_WITH_COV_STAMPED_SPEED"] << "\n";
    }
    oss << "=================================================="; 
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str()  );

  }

};

} // namespace OxTS

#endif // GAD_NODE_HPP