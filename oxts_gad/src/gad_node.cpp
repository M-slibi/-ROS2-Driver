#include "oxts_gad/gad_node.hpp"


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
  gp.SetAidingLeverArmFixed(0.1, 0.1, 0.1);
  gp.SetAidingLeverArmVar(0.01, 0.01, 0.01);

  gad_handler.SendPacket(gp);  
}

void GadNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "odom received");
}




} // namespace OxTS