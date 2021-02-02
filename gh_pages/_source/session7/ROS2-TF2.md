# TF2 - The second generation of the transform library
## Introduction

This tutorial will help you understand the flexibility of transforms with the TF2 library. It will introduce Dynamic Transform Broadcasters, Transform Listeners and Static Transform Broadcasters.   

For this tutorial, we will use the Turtlesim again. This time, we will insert a second turtle into the simulation. As you drive the first turtle around with keyboard teleop, the second turte will follow it closely. In order to do this, the second turtle needs to know where the first one is, w.r.t. its own coordinate frames. This can be easily achieved using the TF2 library.

The example code is based on [tf2_example](https://github.com/ros2/geometry2/tree/eloquent/examples_tf2_py) and is tested with ROS2 Foxy.   



## 1. Dynamic TF Broadcaster

The word dynamic indicates that the transform that is being published is constantly changing. This is useful for tracking moving parts. In this case, we continuously broadcast the current position of the first turtle.

Create an ament python package with dependencies on tf2_ros (Use the "depend" tag). 
```bash 
ros2 pkg create --build-type ament_python <package_name> --dependencies tf2_ros rclpy
``` 

 Create a new python script in this package (under the folder with the package name) and register it as an executable in `setup.py`. For the purpose of this document, the package name is assumed as `tf2_workshop` and executable name as `broadcaster`.  

Make sure the scipy library is installed:
```bash 
pip3 install scipy
``` 

Copy the code shown below into the new file, save it, and build it.   

```python
#! /usr/bin/env python3

import rclpy
import sys

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster
from turtlesim.msg import Pose

class DynamicBroadcaster(Node):

    def __init__(self, turtle_name):
        super().__init__('dynamic_broadcaster')
        self.name_ = turtle_name
        self.get_logger().info("Broadcasting pose of : {}".format(self.name_))
        self.tfb_ = TransformBroadcaster(self)
        self.sub_pose = self.create_subscription(Pose, "{}/pose".format(self.name_), self.handle_pose, 10)

    def handle_pose(self, msg):
        
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id="world"
        tfs._child_frame_id = self.name_
        tfs.transform.translation.x = msg.x
        tfs.transform.translation.y = msg.y
        tfs.transform.translation.z = 0.0  

        r = R.from_euler('xyz',[0,0,msg.theta])

        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]

        self.tfb_.sendTransform(tfs)    

def main(argv=sys.argv[1]):
    rclpy.init(args=argv)
    node = DynamicBroadcaster(sys.argv[1])

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```
Next, add this script to the 

### 1.1 Explanation

This follows the same basic structure for a ROS2 node. The TF2 specific lines will be explained.

```python
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster
```

Imports the modules required for TF2. Scipy is used to convert from Euler angles to quaternion since the `tf_conversions` package has not been ported over to ROS2 yet.

```python
self.tfb_ = TransformBroadcaster(self)
```

Initializes a dynamic transform broadcaster which sends the transforms.

```python
self.name_ = turtle_name
```
The name of the turtle for which we are broadcasting comes from the CLI as an argument.

```python
tfs = TransformStamped()
...
self.tfb_.sendTransform(tfs) 
```

Create a transform datatype with a header and populate it with meaningful data.
The parent frame is always "world" and the child frame (i.e. current frame being published) is the name chosen. Transmit this transformation from within the subscriber callback. This implies it updates the frame (nearly) as fast as it receives the pose.   

* Header
    * Timestamp : 
 (Determine the moment when this transform is happening. This is mainly
rclpy.Time.time() when you want to send the actual transform. This means the
transform can change over time to generate a dynamic motion.)

    * Frame_ID : (The frame ID of the origin frame)

* Child Frame ID: 
 (Frame ID to which the transform is happening)

 * Transform
    * Position : in meter (X, Y and Z)
    * Orientation : in Quaternion (You can use the TF Quaternion from Euler function to use the roll, pitch and yaw angles in rad instead)


### 1.2 Declaring the executable

Now that we have our code written and dependencies setup, we need to tell our build system that the script we created should be treated as an executable. We do this in `setup.py`. Look for the section that starts with `entry_points={`. We edit this part to declare the executable and its entry point to look like this:

```Python
entry_points={
        'console_scripts': [
                'broadcaster = tf2_workshop.broadcaster:main',
        ],
},
```


### 1.3 Testing the Broadcaster

First, start the turtlesim node :   
`ros2 run turtlesim turtlesim_node`   

Then start the broadcaster, with your chosen name for the turtle as the only argument. Here we assume `turtle1`:   

`ros2 run tf2_workshop broadcaster turtle1`   

If all works well, the broadcaster is now sending the TF data for turtle1. This can be verified with:   
`ros2 run tf2_ros tf2_echo turtle1 world`   
or by simply running:   
`ros2 run tf2_ros tf2_monitor`   
or also through rviz2 by adding the `TF` display module.   


## 2. TF Listener

Once your robot system has a fully fleshed out TF tree with valid data at a good frequency, you can then make use of these transforms for your application through listeners, which actually solve inverse kinematics for you. This is the true power of TF2.   

In the following example, we will create a TF listener, which listens to the TF tree to get the transformation between the first and the second turtle. 

Create another new python file for the listener and add it as an executable. This document assumes the name `listener` for it.

Copy the following code into the file and save it:

```python
#!/usr/bin/env python3

import sys
import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

class TfListener(Node):

    def __init__(self, first_turtle, second_turtle):
        super().__init__('tf_listener')
        self.first_name_ = first_turtle
        self.second_name_ = second_turtle
        self.get_logger().info("Transforming from {} to {}".format(self.second_name_, self.first_name_))
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.cmd_ = Twist ()
        self.publisher_ = self.create_publisher(Twist, "{}/cmd_vel".format(self.second_name_),10)
        self.timer = self.create_timer(0.33, self.timer_callback) #30 Hz = 0.333s

    def timer_callback(self):
        try:
            trans = self._tf_buffer.lookup_transform(self.second_name_, self.first_name_, rclpy.time.Time())
            self.cmd_.linear.x = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            self.cmd_.angular.z = 4 * math.atan2(trans.transform.translation.y , trans.transform.translation.x)
            self.publisher_.publish(self.cmd_)

        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))

def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = TfListener(sys.argv[1], sys.argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### 2.1 Explanation

The new lines are explained here.

```python
from tf2_ros.transform_listener import TransformListener   
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException
```

These modules are required for listeners. Recall that the buffer provides an abstracted API of the core TF2 package which is ROS agnostic.   

```python
self._tf_buffer = Buffer()
self._tf_listener = TransformListener(self._tf_buffer, self)
```

Boilerplate code for setting up a new listener.

```python
trans = self._tf_buffer.lookup_transform(self.second_name_, self.first_name_, rclpy.time.Time())
``` 

The main workhorse of this node. The syntax accepts the target frame first and the source frame second. In other words, the pose of the origin of the source frame w.r.t the target frame. It calculates the transformation from the first turtle to the second turtle, which tells us how far away turtle1 is from turtle2.

```python
self.cmd_.linear.x = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
self.cmd_.angular.z = 4 * math.atan2(trans.transform.translation.y , trans.transform.translation.x)
self.publisher_.publish(self.cmd_)
```

This part is a very simply controller, that generates velocity commands based on the distance remaining. This part can be replaced by any other controller you wish to try for the following motion.

Make sure to add the listener as an executable as well (refer to the previous section) and then rebuild the package.

### 2.2 Testing the listener

The turtlesim simulation and teleop node need to be running first.

To test the application, you first need to spawn a second turtle, the name of which is assumed here to be `turtle2`:   
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: "turtle2"}"
``` 

The parameters indicate the starting pose and name.   

You also need to start up another TF broadcaster for this new turtle, similar to the previous one, but with the name of this turtle instead.

Finally, run the executable node for the listener by passing the first argument as the name of the first turtle and the second argument as the name of the second turtle:
```bash
ros2 run tf2_workshop listener turtle1 turtle2
``` 
Now when you move around the first turtle using the keyboard, the second turtle follows it. 

You can once again check the TF tree to observe the new additions.


## 3. Static Transform Broadcaster

The dynamic transform broadcaster is used for moving objects. But for parts that do not move, the simpler static broadcaster does the job well. This could be used for instances like a camera fixed in a room, or a lidar mounted on a mobile robot.

The static publisher is already available as an executable node, and it can simply be started with the right command line arguments which are in order :    

```bash
Translation.x, Translation.y, Translation.z, Rotation.yaw, Rotation.pitch, Rotation.roll, Parent frame, child frame
``` 

For example :
```bash 
ros2 run tf2_ros static_transform_publisher 0.1 0 0 -1.57 0.0 0.0 turtle1 turtle_cam1
``` 

This will add the frame `turtle_cam1` related to the `turtle1` frame.
The virtual camera is mounted +0.1 m in x-axis from view of the turtle base and rotated -1.57 rad in yaw. This might not make much real world sense, but hey it's just an example!

### 3.1 Testing the static broadcaster

The TF published is exactly of the same format as the dynamic publisher, and this can be easily verified with any of the aforementioned means.
