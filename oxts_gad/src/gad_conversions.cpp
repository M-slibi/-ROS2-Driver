#include "oxts_gad/gad_conversions.hpp"



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

  // ROS covariance matrix order is:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  // GAD covariance order for attitude is rot Z (Yaw), rot Y (Pitch), rot X (Roll)
  ga_out.SetAttVar(
    msg->covariance[35],
    msg->covariance[28],
    msg->covariance[21]
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



} // OxTS
