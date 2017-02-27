#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>

#include <fake_ar_publisher/ARMarker.h>

class BaseTest
{
public:
    BaseTest(ros::NodeHandle& nh)
    {
        ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1,
        &BaseTest::ar_callback, this);
    }

    void ar_callback(const fake_ar_publisher::ARMarkerConstPtr& msg)
    {
        last_msg_ = msg;
    }

    fake_ar_publisher::ARMarkerConstPtr get_last_msg(){
      return last_msg_;
    }

    ros::Subscriber ar_sub_;
    fake_ar_publisher::ARMarkerConstPtr last_msg_;
};

TEST(TestSuite, ar_z_position_test)
{
  ros::NodeHandle nh;
  BaseTest basetester(nh);
  std::cout
  ASSERT_TRUE(basetester.get_last_msg()->pose.pose.position.z==0.5);
  ros::spin();
}

TEST(TestSuite, ar_header_frameid_test)
{
  ros::NodeHandle nh;
  BaseTest basetester(nh);
  ASSERT_TRUE(basetester.get_last_msg()->header.frame_id=="camera_frame");
  ros::spin();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "myworkscell_test_node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
