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

void odom_to_gad_position(
  const nav_msgs::msg::Odometry::SharedPtr msg, 
  OxTS::GadPosition& gp_out 
){
  gp_out.SetPosLocal(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z
  );
  gp_out.SetPosLocalVar(
    msg->pose.covariance[0],
    msg->pose.covariance[7],
    msg->pose.covariance[14]
  );
  gp_out.SetTimeVoid();
  gp_out.SetAidingLeverArmConfig();
}


void odom_to_gad_att(
  const nav_msgs::msg::Odometry::SharedPtr msg, 
  OxTS::GadAttitude& ga_out   
){
  auto q_att_enu = tf2::Quaternion();

  tf2::convert(msg->pose.pose.orientation, q_att_enu);


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
    msg->pose.covariance[0],
    msg->pose.covariance[7],
    msg->pose.covariance[14]
  );
  ga_out.SetTimeVoid();
  // Attitude *must* be associated to the KF.
  ga_out.SetAidingAlignmentOptimising();

}

void odom_to_gad_velocity(
  const nav_msgs::msg::Odometry::SharedPtr msg, 
  OxTS::GadVelocity& gv_out 
){
  gv_out.SetVelOdom(
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z
  );
  gv_out.SetVelOdomVar(
    msg->twist.covariance[0],
    msg->twist.covariance[7],
    msg->twist.covariance[14]
  );
  gv_out.SetTimeVoid();
  gv_out.SetAidingLeverArmFixed(0.0,0.0,0.0);
}

void GadNode::nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "%d navsatfix: %lf, %lf, %lf", 
                                  stream_ids["NAV_SAT_FIX_POS"],msg->latitude, msg->longitude, msg->altitude
  );

  OxTS::GadPosition gp = OxTS::GadPosition(stream_ids["NAV_SAT_FIX_POS"]);

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
  // Create and send GAD position
  OxTS::GadPosition gp = OxTS::GadPosition(stream_ids["ODOM_POS"]);
  odom_to_gad_position(msg, gp);
  gad_handler.SendPacket(gp);

  // Create and send GAD attitude
  OxTS::GadAttitude ga = OxTS::GadAttitude(stream_ids["ODOM_ATT"]); 
  odom_to_gad_att(msg, ga);
  gad_handler.SendPacket(ga);

  // Create and send GAD velocity
  OxTS::GadVelocity gv = OxTS::GadVelocity(stream_ids["ODOM_VEL"]);
  odom_to_gad_velocity(msg, gv);
  gad_handler.SendPacket(gv);

  /// Angular velocities not supported by GAD
}




} // namespace OxTS