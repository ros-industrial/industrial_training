/**
 ** Simple ROS Subscriber Node
 **/
#include <ros/ros.h>
//#include <lesson_simple_topic/PathPosition.h> // TODO: Uncomment this line

// TODO: Uncomment the next 5 lines
//void positionCallback(const lesson_simple_topic::PathPosition& msg)
//{
//    ROS_INFO("New position: %.1f,%.1f,%.1f", msg.x, msg.y, 
//        msg.angle * 180.0 / 3.141592);
//}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "simple_subscriber");
    ros::NodeHandle node;

    // TODO: Uncomment the next 2 lines
    //ros::Subscriber subscriber = node.subscribe("position", 1000, positionCallback);
    //ros::spin();
    return 0;
}
