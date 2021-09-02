#include <boost/test/unit_test.hpp>

#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"


#include "oxts_gad/gad_node.hpp"
#include "tests.hpp"

namespace utf = boost::unit_test;
namespace utt = boost::test_tools;


namespace tests::gad_node {

// struct Fixture
// {
//     Fixture()
//     {

//     }

//     ~Fixture()
//     {

//     }
// };



BOOST_AUTO_TEST_SUITE(gad_node)  //, *boost::unit_test::fixture<Fixture>())


BOOST_AUTO_TEST_CASE(odom_to_gad_position_conversion,
    * utf::tolerance(0.1)
) {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->pose.pose.position.x = 1.0;
    msg->pose.pose.position.y = 2.0;
    msg->pose.pose.position.z = 3.0;
    msg->pose.covariance[0]  = 4.0;
    msg->pose.covariance[7]  = 5.0;
    msg->pose.covariance[14] = 6.0;
    int stream_id = 140;
    auto gp = OxTS::GadPosition(stream_id);

    OxTS::odom_to_gad_position(msg,gp);
    std::vector<double> pos_ref = {1.0,2.0,3.0};
    std::vector<double> pos_var_ref = {4.0,5.0,6.0};
    BOOST_TEST(gp.GetPos() == pos_ref,  utt::per_element());
    BOOST_TEST(gp.GetPosVar() == pos_var_ref,  utt::per_element());
}


BOOST_AUTO_TEST_CASE(odom_to_gad_orientation_conversion,
    * utf::tolerance(0.1)
) {
    /// Setup 
    // rclcpp::init(0, nullptr);
    // auto node = std::make_shared<OxTS::GadNode>(rclcpp::NodeOptions());
    int  stream_id = 141;
    auto ga = OxTS::GadAttitude(stream_id); 
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();

    // Pose orientation is a quaternion in ENU. This is converted to 
    // Euler angles in NED
    // Base case
    msg->pose.pose.orientation.x = 0.0;
    msg->pose.pose.orientation.y = 0.0;
    msg->pose.pose.orientation.z = 0.0;
    msg->pose.pose.orientation.w = 1.0;
    OxTS::odom_to_gad_att(msg, ga);
    std::vector<double> att_ref1 = {90.0,0.0,180.0};
    BOOST_TEST(ga.GetAtt() == att_ref1, utt::per_element());
    // Case 2
    msg->pose.pose.orientation.x = 1.0;
    msg->pose.pose.orientation.y = 0.0;
    msg->pose.pose.orientation.z = 0.0;
    msg->pose.pose.orientation.w = 0.0;
    OxTS::odom_to_gad_att(msg, ga);
    std::vector<double> att_ref2 = {90.0,0.0,0.0};
    BOOST_TEST(ga.GetAtt() == att_ref2,  utt::per_element());
    // Case 3
    msg->pose.pose.orientation.x = 0.65328148; 
    msg->pose.pose.orientation.y = 0.65328148;
    msg->pose.pose.orientation.z = 0.27059805;
    msg->pose.pose.orientation.w = -0.27059805;
    OxTS::odom_to_gad_att(msg, ga);
    std::vector<double> att_ref3 = {0.0,45.0,0.0};
    BOOST_TEST(ga.GetAtt() == att_ref3,  utt::per_element());
    // Case 4
    msg->pose.pose.orientation.x = 0.56098553; 
    msg->pose.pose.orientation.y = 0.43045933;
    msg->pose.pose.orientation.z = -0.09229596;
    msg->pose.pose.orientation.w = -0.70105738;
    OxTS::odom_to_gad_att(msg, ga);
    std::vector<double> att_ref4 = {45.0,30.0,90.0};
    BOOST_TEST(ga.GetAtt() == att_ref4,  utt::per_element());
    // TODO: Gimbal lock case??

}

BOOST_AUTO_TEST_SUITE_END()

} // namespace tests
