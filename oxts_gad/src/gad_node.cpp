#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include "tf2_kdl/tf2_kdl.h"

#include "oxts_gad/gad_node.hpp"
#include "oxts_gad/nav_const.hpp"

namespace OxTS
{


void GadNode::nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "navsatfix: %lf, %lf, %lf", 
                                  msg->latitude, msg->longitude, msg->altitude
  );

  OxTS::GadPosition gp = OxTS::GadPosition(130);

  gp.SetPosGeodetic(msg->latitude, msg->longitude, msg->altitude);
  gp.SetPosGeodeticVar(msg->position_covariance[0],
                       msg->position_covariance[4],
                       msg->position_covariance[8]
  );
  gp.SetTimeVoid();
  gp.SetAidingLeverArmConfig();

  gad_handler.SendPacket(gp);
}

void GadNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "odom received");

  // Pose 
  /// Position
  OxTS::GadPosition gp = OxTS::GadPosition(140);
  gp.SetPosLocal(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z
  );
  gp.SetPosLocalVar(
    msg->pose.covariance[0],
    msg->pose.covariance[7],
    msg->pose.covariance[14]
  );
  gp.SetTimeVoid();
  gp.SetAidingLeverArmConfig();

  /// Orientation
  OxTS::GadAttitude ga = OxTS::GadAttitude(141);
  auto q_att = tf2::Quaternion();
  tf2::convert(msg->pose.pose.orientation, q_att);
  auto m_att = tf2::Matrix3x3(q_att);
  double r, p, y;
  m_att.getRPY(r, p, y);
  ga.SetAtt(
    y * NAV_CONST::RADS2DEG,
    p * NAV_CONST::RADS2DEG,
    r * NAV_CONST::RADS2DEG
  );

  // Twist (velocity)
  /// Linear
  msg->twist.twist.linear.x;
  msg->twist.twist.linear.y;
  msg->twist.twist.linear.z;

  /// Angular
  msg->twist.twist.angular.x;
  msg->twist.twist.angular.y;
  msg->twist.twist.angular.z;


  // Twist Covariance
  msg->twist.covariance;


}




} // namespace OxTS