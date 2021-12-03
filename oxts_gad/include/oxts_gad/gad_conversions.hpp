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

static const double UTC_OFFSET_DEFAULT = 18.0;

static std::map<std::string, int> TIMESTAMP_MODE {
  {"void", 0},
  {"latency", 1},
  {"pps", 2},
  {"gps", 3}
};

class GpsTime {
  double week_;
  double secs_;
public:
  GpsTime(){
    week_ = 0.0;
    secs_ = 0.0;
  }
  GpsTime(double week, double secs) : 
    week_(week),
    secs_(secs)
  {}
  bool operator== (const GpsTime &g1){
      return (getWeek() == g1.getWeek() &&
              getSecs() == g1.getSecs());
  }
  double getWeek() const {return week_;}
  double getSecs() const {return secs_;}
  void setWeek(double week) {week_ = week;}
  void setSecs(double secs) {secs_ = secs;}
}; 

GpsTime convert_unix_gps_time(
  const double secs, 
  const double nanosecs,
  const double utc_offset=UTC_OFFSET_DEFAULT
);

template<class T>
void timestamp_gad_from_msg_gps(
  OxTS::Gad& gad,
  const T msg,
  double utc_offset=UTC_OFFSET_DEFAULT
);
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

template<class T>
void timestamp_gad(
  OxTS::Gad& gad,
  T& msg,
  const int timestamp_mode,
  const double utc_offset=UTC_OFFSET_DEFAULT
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
  OxTS::GadAttitude& ga_out,
  const int timestamp_mode,
  const double utc_offset=UTC_OFFSET_DEFAULT
);
/**
 * Convert velocity data in ROS geometry_msgs/TwistWithCovariance to GAD Velocity
 * in an odometry frame
 * @param msg Twist message to convert from
 * @param gv_out GAD object to store converted data in
 */
void twist_with_covariance_to_gad_velocity_odom(
  const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg, 
  OxTS::GadVelocity& gv_out 
);
/**
 * Convert velocity data in ROS geometry_msgs/TwistWithCovariance to GAD Velocity
 * in a local ENU frame
 * @param msg Twist message to convert from
 * @param gv_out GAD object to store converted data in
 */
void twist_with_covariance_to_gad_velocity_local(
  const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg, 
  OxTS::GadVelocity& gv_out 
);
/**
 * Convert 1D velocity data in ROS geometry_msgs/TwistWithCovariance to GAD Speed
 * @param msg Twist message to convert from
 * @param gs_out GAD object to store converted data in
 */
void twist_with_covariance_to_gad_speed(
  const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg, 
  OxTS::GadSpeed& gv_out 
);
/**
 * Convert velocity data in ROS geometry_msgs/TwistWithCovarianceStamped to 
 * GAD Velocity
 * @param msg Twist message to convert from
 * @param gv_out GAD object to store converted data in
 */
void twist_with_covariance_stamped_to_gad_velocity(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg, 
  OxTS::GadVelocity& gv_out,
  int timestamp_mode,
  const double utc_offset=UTC_OFFSET_DEFAULT
);
/**
 * Convert velocity data in ROS geometry_msgs/TwistWithCovarianceStamped to 
 * GAD Speed
 * @param msg Twist message to convert from
 * @param gs_out GAD object to store converted data in
 */
void twist_with_covariance_stamped_to_gad_speed(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg, 
  OxTS::GadSpeed& gs_out,
  int timestamp_mode,
  const double utc_offset=UTC_OFFSET_DEFAULT
);

void odom_to_gad(
  const nav_msgs::msg::Odometry::SharedPtr msg, 
  OxTS::GadPosition& gp_out, 
  OxTS::GadAttitude& ga_out, 
  OxTS::GadVelocity& gv_out, 
  const int timestamp_mode,
  const double utc_offset=UTC_OFFSET_DEFAULT
);

} // namespace OxTS



#endif // GAD_CONVERSIONS_HPP