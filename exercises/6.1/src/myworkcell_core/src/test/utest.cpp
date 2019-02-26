#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fake_ar_publisher/ARMarker.h>

fake_ar_publisher::ARMarkerConstPtr test_msg_;

TEST(TestSuite, myworkcell_core_framework)
{
  ASSERT_TRUE(true);
}

void testCallback(const fake_ar_publisher::ARMarkerConstPtr &msg)
{
  test_msg_ = msg;
}

TEST(TestSuite, myworkcell_core_fake_ar_pub_ref_frame)
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/ar_pose_marker", 1, &testCallback);

    EXPECT_NE(ros::topic::waitForMessage<fake_ar_publisher::ARMarker>("/ar_pose_marker", ros::Duration(10)), nullptr);
    EXPECT_EQ(1, sub.getNumPublishers());
    EXPECT_EQ(test_msg_->header.frame_id, "camera_frame");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "MyWorkcellCoreTest");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
