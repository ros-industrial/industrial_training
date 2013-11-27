/**
 ** Simple ROS Node
 **/
#include <ros/ros.h>
#include <iostream>


int main(int argc, char* argv[])
{
    // This must be called before anything else ROS-related 
    ros::init(argc, argv, "simple_parameters");

    // Create a ROS node handle
    ros::NodeHandle node("~");
    //ros::NodeHandle node("/simple_parameters");   // STEP 2: Change 'node;' to 'node("/simple_parameters");'
    //ros::NodeHandle node("~");                    // STEP 3: Change 'node("/simple_parameters")' to 'node("~");'

    // Set the rate at which we print out our message (1Hz)
    ros::Rate loop_rate(1.0);

    // A simple counter for the number of times we iterate through the loop
    int count = 0;

    // Loop through until the ROS system tells the user to shut down
    while(ros::ok()) {
        // Print out the current iteration
        ROS_INFO_STREAM("Iteration number " << count << ": ");
        ++count;

        // STEP 1: Uncomment the following 8 lines to read out integer parameter
        int integer_value;
        if(!node.hasParam("integer_value")) {
            ROS_WARN_STREAM("  integer_value is not set");
        } else if(!node.getParam("integer_value", integer_value)) {
            ROS_ERROR_STREAM("  integer_value is wrong type");
        } else {
            ROS_INFO_STREAM("  integer_value is " << integer_value);
        }

        // STEP 4: Uncomment the following 6 lines
        double x, y;
        node.param("point/x", x, 0.0);
        node.param("point/y", y, 0.0);
        node.setParam("/global_sum", x + y);
        ROS_INFO_STREAM("  Sum of point values " << x << " & " << y << ": " 
                << x + y);


        // Wait the stated duration
        loop_rate.sleep();
    }

    // Exit the program.
    return 0;
}
