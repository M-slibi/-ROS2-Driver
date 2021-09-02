#include <memory>

#include <oxts/gal-cpp/gad.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include "tf2_kdl/tf2_kdl.h"

#include "oxts_gad/gad_node.hpp"
#include "oxts_gad/nav_const.hpp"

namespace OxTS
{

void pose_with_covariance_to_gad_position(
  const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg, 
  OxTS::GadPosition& gp_out 
){
  gp_out.SetPosLocal(
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z
  );
  gp_out.SetPosLocalVar(
    msg->covariance[0],
    msg->covariance[7],
    msg->covariance[14]
  );
  gp_out.SetTimeVoid();
  gp_out.SetAidingLeverArmConfig();
}


void pose_with_covariance_to_gad_attitude(
  const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg, 
  OxTS::GadAttitude& ga_out   
){
  auto q_att_enu = tf2::Quaternion();
  tf2::convert(msg->pose.orientation, q_att_enu);

  auto q_enu2ned = tf2::Quaternion();
  q_enu2ned.setRPY(
    180.0 * NAV_CONST::DEG2RADS,
    0.0   * NAV_CONST::DEG2RADS,
    90.0  * NAV_CONST::DEG2RADS
  );

  auto q_att =  q_enu2ned * q_att_enu;
  auto m_att = tf2::Matrix3x3(q_att);

  double r, p, y;
  m_att.getRPY(r, p, y);
  ga_out.SetAtt(
    y * NAV_CONST::RADS2DEG,
    p * NAV_CONST::RADS2DEG,
    r * NAV_CONST::RADS2DEG
  );

  ga_out.SetAttVar(
    msg->covariance[0],
    msg->covariance[7],
    msg->covariance[14]
  );
  ga_out.SetTimeVoid();
  // Attitude *must* be associated to the KF.
  ga_out.SetAidingAlignmentOptimising();

}

void pose_with_covariance_to_gad(
  const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg, 
  OxTS::GadPosition& gp_out, 
  OxTS::GadAttitude& ga_out 
){
  pose_with_covariance_to_gad_position(msg, gp_out);
  pose_with_covariance_to_gad_attitude(msg, ga_out);
}

void pose_with_covariance_stamped_to_gad(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, 
  OxTS::GadPosition& gp_out, 
  OxTS::GadAttitude& ga_out 
){
  auto pose = 
    std::make_shared<geometry_msgs::msg::PoseWithCovariance>(msg->pose);
  pose_with_covariance_to_gad(pose, gp_out, ga_out);
  /** TODO: use timestamp */
}

void twist_with_covariance_to_gad_velocity(
  const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg, 
  OxTS::GadVelocity& gv_out 
){
  gv_out.SetVelOdom(
    msg->twist.linear.x,
    msg->twist.linear.y,
    msg->twist.linear.z
  );
  gv_out.SetVelOdomVar(
    msg->covariance[0],
    msg->covariance[7],
    msg->covariance[14]
  );
  gv_out.SetTimeVoid();
  gv_out.SetAidingLeverArmFixed(0.0,0.0,0.0);
}

void twist_with_covariance_stamped_to_gad(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg, 
  OxTS::GadVelocity& gv_out 
){
  auto twist = 
    std::make_shared<geometry_msgs::msg::TwistWithCovariance>(msg->twist);
  twist_with_covariance_to_gad_velocity(twist, gv_out);
  /** TODO: use timestamp */
}

void GadNode::nav_sat_fix_callback(
  const sensor_msgs::msg::NavSatFix::SharedPtr msg
){
  RCLCPP_INFO(this->get_logger(), "%d navsatfix: %lf, %lf, %lf", 
              stream_ids["NAV_SAT_FIX_POS"],
              msg->latitude,
              msg->longitude,
              msg->altitude
  );

  auto gp = OxTS::GadPosition(stream_ids["NAV_SAT_FIX_POS"]);

  gp.SetPosGeodetic(msg->latitude, msg->longitude, msg->altitude);
  gp.SetPosGeodeticVar(msg->position_covariance[0],
                       msg->position_covariance[4],
                       msg->position_covariance[8]
  );
  gp.SetTimeVoid();
  gp.SetAidingLeverArmConfig();

  gad_handler.SendPacket(gp);
}

void odom_to_gad(
  const nav_msgs::msg::Odometry::SharedPtr msg, 
  OxTS::GadPosition& gp_out, 
  OxTS::GadAttitude& ga_out, 
  OxTS::GadVelocity& gv_out 
){
  auto pose =
    std::make_shared<geometry_msgs::msg::PoseWithCovariance>(msg->pose);
  auto twist =
    std::make_shared<geometry_msgs::msg::TwistWithCovariance>(msg->twist);
  pose_with_covariance_to_gad(pose, gp_out, ga_out);
  twist_with_covariance_to_gad_velocity(twist, gv_out);
}

void GadNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Create GAD
  auto gp = OxTS::GadPosition(stream_ids["ODOM_POS"]);
  auto ga = OxTS::GadAttitude(stream_ids["ODOM_ATT"]); 
  auto gv = OxTS::GadVelocity(stream_ids["ODOM_VEL"]);
  // Convert odometry message into the three aiding types (angular velocities 
  // in odom cannot be used by GAD)
  odom_to_gad(msg, gp, ga, gv);
  // Send GAD to file or unit, depending on config
  gad_handler.SendPacket(gp);
  gad_handler.SendPacket(ga);
  gad_handler.SendPacket(gv);
}

void GadNode::pose_with_cov_stamped_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg
){
  auto gp = OxTS::GadPosition(stream_ids["POSE_WITH_COV_STAMPED_POS"]);
  auto ga = OxTS::GadAttitude(stream_ids["POSE_WITH_COV_STAMPED_ATT"]); 
  auto pose = 
    std::make_shared<geometry_msgs::msg::PoseWithCovariance>(msg->pose);
  // Convert pose into position and attitude aiding types 
  pose_with_covariance_stamped_to_gad(msg, gp, ga);
  // Send GAD to file or unit, depending on config
  gad_handler.SendPacket(gp);
  gad_handler.SendPacket(ga);
}

void GadNode::twist_with_cov_stamped_callback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg
){
  auto gv = OxTS::GadVelocity(stream_ids["TWIST_WITH_COV_STAMPED_VEL"]);

  // Convert twist into velocity aiding (angular rates not supported by GAD) 
  twist_with_covariance_stamped_to_gad(msg, gv);
  // Send GAD to file or unit, depending on config
  gad_handler.SendPacket(gv);
}



} // namespace OxTS