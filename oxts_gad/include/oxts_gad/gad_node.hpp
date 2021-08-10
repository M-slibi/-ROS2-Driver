/**
 * \file tests.h
 * Contains anything which is common to all the unit tests.
 */
#ifndef GAD_NODE_HPP
#define GAD_NODE_HPP

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "oxts/gal-cpp/gad.hpp"
#include "oxts/gal-cpp/gad_handler.hpp"


namespace OxTS
{

class GadNode : public rclcpp::Node
{
  public:
  // Parameters
  std::string unit_ip;
  std::string file_out;
  std::string nav_sat_fix_topic;
  std::string odom_topic;
  std::vector<double> lva;
  std::vector<double> lva_var;
  
  // Other class members
  OxTS::GadHandler gad_handler; 

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subNavSatFix_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;



  // Constructor
  GadNode(const rclcpp::NodeOptions &options)
  : Node("oxts_gad", options)
  {
    // Load parameters
    unit_ip           = this->declare_parameter("unit_ip", std::string(""));
    file_out          = this->declare_parameter("file_out", "~/out.gad");
    nav_sat_fix_topic = this->declare_parameter("nav_sat_fix_topic", "/ins/nav_sat_fix");
    odom_topic        = this->declare_parameter("odom_topic", "/ins/odom");

    // Relay any config that might be useful to the command line
    RCLCPP_INFO(this->get_logger(), "INS IP: %s", unit_ip.c_str());



    gad_handler.SetEncoderToBin();
    gad_handler.SetOutputModeToUdp(unit_ip);


    subNavSatFix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        nav_sat_fix_topic, 
        10, 
        std::bind(&GadNode::nav_sat_fix_callback, this, std::placeholders::_1)
    );
    subOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 
        10, 
        std::bind(&GadNode::odometry_callback, this, std::placeholders::_1)
    );
  }

  private:

  void nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

};

} // namespace OxTS

#endif // GAD_NODE_HPP