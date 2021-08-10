/**
 * \file tests.h
 * Contains anything which is common to all the unit tests.
 */
#ifndef GAD_NODE_HPP
#define GAD_NODE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "oxts/gal-cpp/gad.hpp"
#include "oxts/gal-cpp/gad_handler.hpp"


class OxtsGadNode : public rclcpp::Node
{
  public:
  // Configurables
  std::string unit_ip = "192.168.25.217";
  std::string file_out = "out.gad";
  
  
  // Other class members
  OxTS::GadHandler gad_handler; 

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subNavSatFix_;



  // Constructor
  OxtsGadNode()
  : Node("oxts_gad")
  {
    gad_handler.SetEncoderToBin();
    gad_handler.SetOutputModeToUdp(unit_ip);


    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "this_topic", 10, std::bind(&OxtsGadNode::topic_callback, this, std::placeholders::_1));
    subNavSatFix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/ins/nav_sat_fix", 10, std::bind(&OxtsGadNode::nav_sat_fix_callback, this, std::placeholders::_1));
  }

  private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  void nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Got a navsatfix");
    
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

};

#endif // GAD_NODE_HPP