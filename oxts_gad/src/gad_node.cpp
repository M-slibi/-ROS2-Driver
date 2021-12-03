#include <memory>

#include <oxts/gal-cpp/gad.hpp>

#include "oxts_gad/gad_node.hpp"
#include "oxts_gad/nav_const.hpp"

namespace OxTS
{

void GadNode::nav_sat_fix_callback(
  const sensor_msgs::msg::NavSatFix::SharedPtr msg
){
  // RCLCPP_INFO(this->get_logger(), "%d navsatfix: %lf, %lf, %lf", 
  //             stream_ids["NAV_SAT_FIX_POS"],
  //             msg->latitude,
  //             msg->longitude,
  //             msg->altitude
  // );

  auto gp = OxTS::GadPosition(stream_ids["NAV_SAT_FIX_POS"]);

  gp.SetPosGeodetic(msg->latitude, msg->longitude, msg->altitude);
  gp.SetPosGeodeticVar(
    msg->position_covariance[0],
    msg->position_covariance[4],
    msg->position_covariance[8]
  );
  gp.SetTimeVoid();
  gp.SetAidingLeverArmConfig();

  gad_handler.SendPacket(gp);
}


void GadNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Create GAD
  auto gp = OxTS::GadPosition(stream_ids["ODOM_POS"]);
  auto ga = OxTS::GadAttitude(stream_ids["ODOM_ATT"]); 
  auto gv = OxTS::GadVelocity(stream_ids["ODOM_VEL"]);
  // Convert odometry message into the three aiding types (angular velocities 
  // in odom cannot be used by GAD)
  odom_to_gad(msg, gp, ga, gv, this->timestamp_mode, this->utc_offset);
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
  pose_with_covariance_stamped_to_gad(msg, gp, ga, this->timestamp_mode, this->utc_offset);
  // Send GAD to file or unit, depending on config
  gad_handler.SendPacket(gp);
  gad_handler.SendPacket(ga);
}

void GadNode::twist_with_cov_stamped_callback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg
){
  auto gv = OxTS::GadVelocity(stream_ids["TWIST_WITH_COV_STAMPED_VEL"]);
  // Convert twist into velocity aiding (angular rates not supported by GAD) 
  twist_with_covariance_stamped_to_gad_velocity(msg, gv, this->timestamp_mode, this->utc_offset);
  // Send GAD to file or unit, depending on config
  gad_handler.SendPacket(gv);
}



} // namespace OxTS