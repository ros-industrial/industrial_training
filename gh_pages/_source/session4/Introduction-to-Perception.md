# Introduction to Perception
>In this exercise, we will experiment with data generated from the Asus Xtion Pro (or Microsoft Kinect) sensor in order to become more familiar with processing 3D data. We will view its data stream and visualize the data in various ways under Rviz.

## Point Cloud Data File
The start of most perception processing is ROS message data from a sensor.  In this exercise, we'll be using 3D [point cloud](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) data from a common Kinect-style sensor.

1. First, publish the point cloud data as a ROS message to allow display in rviz.

   1. Start `roscore` running in a terminal.
   1. Create a new directory for this exercise:
      ```
      mkdir ~/ex4.2
      cd ~/ex4.2
      cp ~/industrial_training/exercises/4.2/table.pcd .
      ```
      
   1. Publish pointcloud messages from the pre-recorded `table.pcd` point cloud data file:
      ```
      cd ~/ex4.2
      rosrun pcl_ros pcd_to_pointcloud table.pcd 0.1 _frame_id:=map cloud_pcd:=orig_cloud_pcd
      ```
   1. Verify that the `orig_cloud_pcd` topic is being published: `rostopic list`

## Display the point cloud in RViz

1. Start an RViz window, to display the results of point-cloud processing
   ```
   rosrun rviz rviz
   ```

1. Add a **PointCloud2** display item and set the desired topic.
   1. Select **Add** at the bottom of the Displays panel
   1. Select **PointCloud2**
   1. Expand **PointCloud2** in the display tree, and select a topic from topic drop down.
      * _Hint: If you are using the point cloud file, the desired topic is `/orig_cloud_pcd`._

## Experiment with PCL
   Next, we will experiment with various command line tool provided by PCL for processing point cloud data. There are over 140 command line tools available, but only a few will be used as part of this exercise. The intent is to get you familiar with the capabilities of PCL without writing any code, but these command line tools are a great place to start when writing your own.  Although command line tools are helpful for testing various processing methods, most applications typically use the C++ libraries directly for "real" processing pipelines.  The ROS-I Advanced training course explores these C++ PCL methods in more detail.
   
   Each of the PCL commands below generates a new point cloud file (`.pcd`) with the result of the PCL processing command.  Use either the `pcl_viewer` to view the results directly or the `pcd_to_pointcloud` command to publish the point cloud data as a ROS message for display in rviz.  Feel free to stop the `pcd_to_pointcloud` command after reviewing the results in rviz.

### Downsample the point cloud using the pcl_voxel_grid.

1. Downsample the original point cloud using a voxel grid with a grid size of (0.05,0.05,0.05).  In a voxel grid, all points in a single grid cube are replaced with a single point at the center of the voxel.  This is a common method to simplify overly complex/detailed sensor data, to speed up processing steps.
  ```
  pcl_voxel_grid table.pcd table_downsampled.pcd -leaf 0.05,0.05,0.05
  pcl_viewer table_downsampled.pcd
  ```

1. View the new point cloud in rviz.(optional)
  ```
  rosrun pcl_ros pcd_to_pointcloud table_downsampled.pcd 0.1 _frame_id:=map cloud_pcd:=table_downsampled
  ```
  Note: For the PointCloud2 in rviz change the topic to */table_downsampled* to show the new data.

### Extracting the table surface from point cloud using the pcl_sac_segmentation_plane.

1.  Find the largest plane and extract points that belong to that plane (within a given threshold).
  ```
  pcl_sac_segmentation_plane table_downsampled.pcd only_table.pcd -thresh 0.01
  pcl_viewer only_table.pcd
  ```
  View the new point cloud in rviz.(optional)
  ```
  rosrun pcl_ros pcd_to_pointcloud only_table.pcd 0.1 _frame_id:=map cloud_pcd:=only_table
  ```
  Note: For the PointCloud2 in rviz change the topic to */only_table* to show the new data.
  
### Extracting the largest cluster on the table from point cloud using the pcl_sac_segmentation_plane.

1.  Extract the largest point-cluster not belonging to the table.
  ```
  pcl_sac_segmentation_plane table.pcd object_on_table.pcd -thresh 0.01 -neg 1
  pcl_viewer object_on_table.pcd
  ```
  View the new point cloud in rviz.(optional)
  ```
  rosrun pcl_ros pcd_to_pointcloud object_on_table.pcd 0.1 _frame_id:=map cloud_pcd:=object_on_table
  ```
  Note: For the PointCloud2 in rviz change the topic to */object_on_table* to show the new data.

### Remove outliers from the cloud using the pcl_outlier_removal.
1. For this example, a statistical method will be used for removing outliers.  This is useful to clean up noisy sensor data, removing false artifacts before further processing.
  ```
  pcl_outlier_removal table.pcd table_outlier_removal.pcd -method statistical
  pcl_viewer table_outlier_removal.pcd
  ```
1. View the new point cloud in rviz. (optional)
  ```
  rosrun pcl_ros pcd_to_pointcloud table_outlier_removal.pcd 0.1 _frame_id:=map cloud_pcd:=table_outlier_removal
  ```
  Note: For the PointCloud2 in rviz change the topic to */table_outlier_removal* to show the new data.

### Compute the normals for each point in the point cloud using the pcl_normal_estimation.
1. This example estimates the local surface normal (perpendicular) vectors at each point.  For each point, the algorithm uses nearby points (within the specified radius) to fit a plane and calculate the normal vector.  Zoom in to view the normal vectors in more detail.
  ```
  pcl_normal_estimation only_table.pcd table_normals.pcd -radius 0.1
  pcl_viewer table_normals.pcd -normals 10
  ```

### Mesh a point cloud using the marching cubes reconstruction.
Point cloud data is often unstructured, but sometimes processing algorithms need to operate on a more structured surface mesh.  This example uses the "marching cubes" algorithm to construct a surface mesh that approximates the point cloud data.
  ```
  pcl_marching_cubes_reconstruction table_normals.pcd table_mesh.vtk -grid_res 20
  pcl_viewer table_mesh.vtk
  ```
