#include <boost/test/unit_test.hpp>

#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "oxts_gad/gad_conversions.hpp"
#include "tests.hpp"

namespace utf = boost::unit_test;
namespace utt = boost::test_tools;


namespace tests::gad_conversions {

// struct Fixture
// {
//     Fixture()
//     {

//     }

//     ~Fixture()
//     {

//     }
// };



BOOST_AUTO_TEST_SUITE(gad_conversions)  //, *boost::unit_test::fixture<Fixture>())


BOOST_AUTO_TEST_CASE(convert_unix_gps_time,
    * utf::tolerance(0.1)
){
    double secs = 1631607897;
    double nanosecs = 400000000; // (0.4s)
    double utc_offset = 18;
    OxTS::GpsTime time = OxTS::convert_unix_gps_time(
        secs, 
        nanosecs,
        utc_offset
    );    
    OxTS::GpsTime reference_time(2175, 203115.4);
    
    BOOST_TEST((time == reference_time));
}

BOOST_AUTO_TEST_CASE(pose_with_covariance_to_gad_position_conversion,
    * utf::tolerance(0.1)
) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovariance>();
    msg->pose.position.x = 1.0;
    msg->pose.position.y = 2.0;
    msg->pose.position.z = 3.0;
    msg->covariance[0]  = 4.0;
    msg->covariance[7]  = 5.0;
    msg->covariance[14] = 6.0;
    int stream_id = 140;
    auto gp = OxTS::GadPosition(stream_id);
    
    OxTS::pose_with_covariance_to_gad_position(msg,gp);
    std::vector<double> pos_ref = {1.0,2.0,3.0};
    std::vector<double> pos_var_ref = {4.0,5.0,6.0};
    BOOST_TEST(gp.GetPos() == pos_ref,  utt::per_element());
    BOOST_TEST(gp.GetPosVar() == pos_var_ref,  utt::per_element());
}


BOOST_AUTO_TEST_CASE(odom_to_gad_orientation_conversion,
    * utf::tolerance(0.1)
) {
    /// Setup 
    int  stream_id = 141;
    auto ga = OxTS::GadAttitude(stream_id); 
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovariance>();

    // Pose orientation is a quaternion in ENU. This is converted to 
    // Euler angles in NED
    // Base case
    msg->pose.orientation.x = 0.0;
    msg->pose.orientation.y = 0.0;
    msg->pose.orientation.z = 0.0;
    msg->pose.orientation.w = 1.0;
    OxTS::pose_with_covariance_to_gad_attitude(msg, ga);
    std::vector<double> att_ref1 = {90.0,0.0,180.0};
    BOOST_TEST(ga.GetAtt() == att_ref1, utt::per_element());
    // Case 2
    msg->pose.orientation.x = 1.0;
    msg->pose.orientation.y = 0.0;
    msg->pose.orientation.z = 0.0;
    msg->pose.orientation.w = 0.0;
    OxTS::pose_with_covariance_to_gad_attitude(msg, ga);
    std::vector<double> att_ref2 = {90.0,0.0,0.0};
    BOOST_TEST(ga.GetAtt() == att_ref2,  utt::per_element());
    // Case 3
    msg->pose.orientation.x = 0.65328148; 
    msg->pose.orientation.y = 0.65328148;
    msg->pose.orientation.z = 0.27059805;
    msg->pose.orientation.w = -0.27059805;
    OxTS::pose_with_covariance_to_gad_attitude(msg, ga);
    std::vector<double> att_ref3 = {0.0,45.0,0.0};
    BOOST_TEST(ga.GetAtt() == att_ref3,  utt::per_element());
    // Case 4
    msg->pose.orientation.x = 0.56098553; 
    msg->pose.orientation.y = 0.43045933;
    msg->pose.orientation.z = -0.09229596;
    msg->pose.orientation.w = -0.70105738;
    OxTS::pose_with_covariance_to_gad_attitude(msg, ga);
    std::vector<double> att_ref4 = {45.0,30.0,90.0};
    BOOST_TEST(ga.GetAtt() == att_ref4,  utt::per_element());
    // TODO: Gimbal lock case??

}

BOOST_AUTO_TEST_CASE(twist_with_covariance_to_gad_velocity_odom_conversion,
    * utf::tolerance(0.1)
) {
    // Setup
    auto msg = std::make_shared<geometry_msgs::msg::TwistWithCovariance>();
    msg->twist.linear.x = 1.0;
    msg->twist.linear.y = 2.0;
    msg->twist.linear.z = 3.0;
    msg->covariance[0] = 4.0;
    msg->covariance[7] = 5.0;
    msg->covariance[14] = 6.0;
    int stream_id = 143;
    auto gv = OxTS::GadVelocity(stream_id);
    // Execute
    OxTS::twist_with_covariance_to_gad_velocity_odom(msg,gv);
    // Test
    std::vector<double> vel_ref = {1.0,2.0,3.0};
    std::vector<double> vel_var_ref = {4.0,5.0,6.0};
    BOOST_TEST(gv.GetVel() == vel_ref,  utt::per_element());
    BOOST_TEST(gv.GetVelVar() == vel_var_ref,  utt::per_element());
}


BOOST_AUTO_TEST_SUITE_END()

} // namespace tests
