#ifndef GAD_CONVERSIONS_HPP
#define GAD_CONVERSIONS_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "oxts/gal-cpp/gad.hpp"
#include "oxts/gal-cpp/gad_handler.hpp"

#include "oxts_gad/nav_conversions.hpp"


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

void odom_to_gad(
  const nav_msgs::msg::Odometry::SharedPtr msg, 
  OxTS::GadPosition& gp_out, 
  OxTS::GadAttitude& ga_out, 
  OxTS::GadVelocity& gv_out 
);

} // namespace OxTS



#endif // GAD_CONVERSIONS_HPP