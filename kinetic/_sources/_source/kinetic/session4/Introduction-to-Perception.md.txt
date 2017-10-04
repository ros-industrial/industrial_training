# Introduction to Perception
>In this exercise, we will experiment with data generated from the asus xiton pro(kinect) sensor in order to become more familiar with process 3D data. We will view its data stream and visualize the data in various ways under Rviz. 

## Kinect Sensor Example
>This task will step you through the process of connecting to and displaying kinect data.

Download the [PointCloud file](resources/table.pcd) and place the file in your home directory (~).

## Launch Point Cloud Generating Nodes
If using a virtual machine, or otherwise don't have access to hardware, you will have to play the data using the PointCloud file and pcl_ros package.

* Terminal 1:
  ```
  roscore
  ```
* Terminal 2:
  ```
  cd ~
  rosrun pcl_ros pcd_to_pointcloud table.pcd 0.1 _frame_id:=map cloud_pcd:=orig_cloud_pcd
  ```

If using OpenNI (Kinect or Asus) device, launch openni driver nodes. (Optional)

```
roslaunch openni_launch openni.launch
```

## Execute rviz

```
rosrun rviz rviz
```

## dd a Point Cloud
Add a **PointCloud2** display item and set the desired topic.

1. Select **Add** at the bottom of the Displays panel
1. Select **PointCloud2**
1. Expand **PointCloud2** in the display tree, and select a topic from topic drop down.
#### Hint:
* If using PointCloud file, the desired topic is */orig_cloud_pcd*.

## Experiment with PCL
   Next step we will experiment with various command line tool provided by PCL for process point cloud data. Now there are over 140 command line tools available so only a few will be used as part of this exercise. The intent is to get you familiar with the capabilities of PCL without writing any code, but these command line tools are a great place to start when writing your own.

### Extracting the table surface from point cloud using the pcl_sac_segmentation_plane.

* Terminal 3:

  Extract only the points that are on a plane within a given threshold.
  ```
  cd ~
  pcl_sac_segmentation_plane table.pcd only_table.pcd -thresh 0.01
  pcl_viewer only_table.pcd
  ```
  View the new point cloud in rviz.(optional)
  ```
  rosrun pcl_ros pcd_to_pointcloud only_table.pcd 0.1 _frame_id:=map cloud_pcd:=only_table
  ```
  Note: For the PointCloud2 in rviz change the topic to */only_table* to show the new data.
  
### Extracting the largest cluster on the table from point cloud using the pcl_sac_segmentation_plane.

* Terminal 3:

  Extract the largest cluster on the table.
  ```
  cd ~
  pcl_sac_segmentation_plane table.pcd object_on_table.pcd -thresh 0.01 -neg 1
  pcl_viewer object_on_table.pcd
  ```
  View the new point cloud in rviz.(optional)
  ```
  rosrun pcl_ros pcd_to_pointcloud object_on_table.pcd 0.1 _frame_id:=map cloud_pcd:=object_on_table
  ```
  Note: For the PointCloud2 in rviz change the topic to */object_on_table* to show the new data.

### Down sample the point cloud using the pcl_voxel_grid.

* Terminal 3:

  Down sample by creating a voxel grid with a leaf size of (0.05,0.05,0.05), where for every point that are in a single leaf are combined into a single point.
  ```
  cd ~
  pcl_voxel_grid table.pcd table_downsampled.pcd -leaf 0.05,0.05,0.05
  pcl_viewer table_downsampled.pcd
  ```
  View the new point cloud in rviz.(optional)
  ```
  rosrun pcl_ros pcd_to_pointcloud table_downsampled.pcd 0.1 _frame_id:=map cloud_pcd:=table_downsampled
  ```
  Note: For the PointCloud2 in rviz change the topic to */table_downsampled* to show the new data.

### Remove outliers from the cloud using the pcl_outlier_removal.

* Terminal 3:

  For this example, a statistical method will be used for removing outliers.
  ```
  cd ~
  pcl_outlier_removal table.pcd table_outlier_removal.pcd -method statistical
  pcl_viewer table_outlier_removal.pcd
  ```
  View the new point cloud in rviz. (optional)
  ```
  rosrun pcl_ros pcd_to_pointcloud table_outlier_removal.pcd 0.1 _frame_id:=map cloud_pcd:=table_outlier_removal
  ```
  Note: For the PointCloud2 in rviz change the topic to */table_outlier_removal* to show the new data.

### Compute the normals for each point in the point cloud using the pcl_normal_estimation.

* Terminal 3:

  This uses neighboring points to fit a plane and the normal of the plane is assigned to the point. It this example a radius is specified, where it will use all the point within a radius around a given point for fitting the plane.
  ```
  cd ~
  pcl_normal_estimation only_table.pcd table_normals.pcd -radius 0.1
  pcl_viewer table_normals.pcd -normals 10
  ```

### Mesh a point cloud using the marching cubes reconstruction.

* Terminal 3:

  ```
  cd ~
  pcl_marching_cubes_reconstruction table_normals.pcd table_mesh.vtk -grid_res 50
  pcl_viewer table_mesh.vtk
  ```

**Note:** If you are going to move on to exercise 4.3, you should save over the default Rviz config, or save your new configuration.
