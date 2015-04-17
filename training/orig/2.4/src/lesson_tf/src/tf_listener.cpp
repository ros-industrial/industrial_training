#include <ros/ros.h>
#include <tf/tf.h>
//#include <tf/transform_listener.h>
//#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv) {

    //Set up the node and nodehandle.
    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Started node tf_listener.");
  
    std::string target_frame, source_frame;
//    tf::TransformListener tf_listener;
//    tf::StampedTransform transform;
    ros::Rate rate(1.); //used to throttle execution
    
    if (argc < 3) {//if no arguments are given, use base_link and part as the default frame names
        target_frame = "/base_link";
        source_frame = "/part";
    }
    else {//if there are at least 2 arguments given, use them as frame names
        target_frame = argv[1];
        source_frame = argv[2];
    }    
        
    while (ros::ok()) {
        try {
            //find transform from target TO source (naming is a bit confusing),
            //at the current time (ros::Time()), and assign result to transform.
//            tf_listener.lookupTransform(target_frame, source_frame, ros::Time(), transform);
            
            //copy transform to a msg type to utilize stringstream properties
            //show transform, and distance between two transforms
            geometry_msgs::Transform buffer;
            tf::transformTFToMsg(transform, buffer);
            ROS_INFO_STREAM("Transform from " << target_frame << " to " << source_frame << ": " << std::endl << buffer);
//            ROS_INFO_STREAM("Distance between transforms: " << transform.getOrigin().length() << " meters.");
        }
        catch (...) {//assume that the exception thrown is because transform is not available yet
            ROS_WARN_STREAM("Waiting for transform from " << target_frame << " to " << source_frame);
        }
        rate.sleep();   //throttle execution (1 second default)
    }
    return 0;
}
