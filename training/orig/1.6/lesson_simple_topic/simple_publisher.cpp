/**
 ** Simple ROS Publisher Node
 **/
#include <ros/ros.h>
//#include <lesson_simple_topic/PathPosition.h> // TODO: Uncomment this line


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "simple_publisher");
    ros::NodeHandle node;

    // Create a 1Hz update rate
    ros::Rate loop_rate(1.0);

    // Advertise that we're publishing the topic "position", of type PathPosition
    //ros::Publisher pub = node.advertise<lesson_simple_topic::PathPosition>("position", 1000); // TODO: Uncomment this line
    
    // The angle counter
    int angle = 0; 

    ROS_INFO("Starting publisher");
    while(ros::ok()) {
        // Create a message, and set the fields appropriately 
        // TODO: Uncomment the next 4 lines
        //lesson_simple_topic::PathPosition msg;
        //msg.angle = angle * 3.141592 / 180.0;
        //msg.x = 100.0 * cos(msg.angle);
        //msg.y = 100.0 * sin(msg.angle);
        // Update angle
        angle = (angle + 10) % 360;

        // Publish the message, give ROS an opportunity to run
        // TODO: Uncomment the next 3 lines
        //pub.publish(msg);
        //ros::spinOnce();
        //ROS_INFO("Published message %.1f, %.1f, %.1f", msg.x, msg.y, msg.angle * 180.0 / 3.141592);
        // Wait 1 second to publish again
        loop_rate.sleep();
    }

    ROS_INFO("Publisher done.");
    return 0;
}
