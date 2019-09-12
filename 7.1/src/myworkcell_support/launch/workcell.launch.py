# <launch>
#   <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" output="screen"/>
#   <node name="vision_node" pkg="myworkcell_core" type="vision_node" output="screen"/>
#   <node name="myworkcell_node" pkg="myworkcell_core" type="myworkcell_node" output="screen">
#     <param name="base_frame" value="world"/>
#   </node>
# </launch>

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
  return LaunchDescription([
    launch_ros.actions.Node(
      node_name='fake_ar_publisher',
      package='fake_ar_publisher',
      node_executable='fake_ar_publisher_node',
      output='screen',
    ),
    launch_ros.actions.Node(
      node_name='vision_node',
      package='myworkcell_core',
      node_executable='vision_node',
      output='screen',
    ),
    launch_ros.actions.Node(
      node_name='myworkcell_node',
      package='myworkcell_core',
      node_executable='myworkcell_node',
      output='screen',
      parameters=[{'base_frame': 'world'}],
    )
  ])
