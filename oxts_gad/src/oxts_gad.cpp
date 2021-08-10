/**
 * \file oxts_gad.cpp
 * Runs the OxtsGadNode.
 */


#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "oxts_gad/gad_node.hpp"

  #include "oxts/gal-cpp/gad.hpp"
#include "oxts/gal-cpp/gad_handler.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OxtsGadNode>());
  rclcpp::shutdown();
  return 0;
}