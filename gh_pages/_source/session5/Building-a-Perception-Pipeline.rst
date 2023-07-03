Building a Perception Pipeline
==============================
In this exercise, we will fill in the appropriate pieces of code to build a perception pipeline. The end goal will be to broadcast a transform with the pose information of the object of interest.

Prepare New Workspace:
----------------------
We will create a new workspace, since this exercise does not overlap with the previous exercises.

#. Disable automatic sourcing of your previous workspace in your ``.bashrc`` if you had any:

      .. Note:: This means you'll need to manually source the setup file from your new colcon workspace in each new terminal window.

#. Source ROS 2 into your environment

   .. code-block:: bash

			source /opt/ros/humble/setup.bash

#. Copy the template workspace layout and files (if you haven't done exercise 4.1):

   .. code-block:: bash

      mkdir -p ~/perception_ws/src
      cp -r ~/industrial_training/exercises/5.1/template_ws/ros2/lesson_perception ~/perception_ws/src
      cd ~/perception_ws/

#. Initialize and Build this new workspace

   .. code-block:: bash

			colcon build

#. Source the workspace

   .. code-block:: bash

      source ~/perception_ws/install/setup.bash

#. Copy the PointCloud file from prior Exercise 4.2:

   .. code-block:: bash

      cp ~/industrial_training/exercises/4.2/table.pcd ~

#. Import the new workspace into your QTCreator IDE (if using QTCreator):

   * In QTCreator: `File -> New File or Project -> Other Project -> ROS Workspace -> ~/perception_ws`
   * Don't forget to check the ROS distro and the build system!

Intro (Review Existing Code)
----------------------------
Most of the infrastructure for a ROS node has already been completed for you; the focus of this exercise is the perception algorithms/pipleline. The `CMakelists.txt` and `package.xml` are complete and an executable has been provided. You could run the executable as is, but you would get errors. At this time we will explore the source code that has been provided - browse the provided `perception_node.cpp` file. The following are highlights of what is included.

#. Headers:

   * We have provided several headers for the various libraries and packages you will need.

#. int main():

   * The ``main`` function has been provided.

#. class PerceptionNode():

   * A ``PerceptionNode()`` class that inherits from ``rclcpp::Node`` has been provided. Parts of it will need to be edited as you go.

#. ROS initialization:

   * Both ``rclcpp::init`` and ``rclcpp::spin`` have been called/initialized.

#. Launch file:

   * A launch file has been provided for you. The parameters within it will need to be edited later in the exercise. 

#. Set up parameters:

   * Currently there are three string parameters included in the example: the world frame, the camera frame and the topic being published by the camera. More parameters will be added later.

#. Set up publishers:

   * Two publishers have been set up to publish ros messages for point clouds. It is often useful to visualize your results when working with image or point cloud processing.

#. Listen for PointCloud2:

   * A subscriber has been set up to listen for new point clouds.

#. Transform PointCloud2 (within callback):

   * While we could work in the camera frame, things are more understandable/useful if we are looking at the points of a point cloud in an xyz space that makes more sense with our environment. In this case we are transforming the points from the camera frame to a world frame.

#. Convert PointCloud2 (ROS to PCL) (within callback)

#. Convert PointCloud2 (PCL to ROS) and publish (within callback):

   * This step is not necessary, but visualizing point cloud processing results is often useful, so conversion back into a ROS type and creating the ROS message for publishing is done for you.

So it seems that a lot has been done! Should be easy to finish up. All you need to do is fill in the middle section.

Primary Task: Filling in the blanks
-----------------------------------
The task of filling in the middle section containing the perception algorithms is an iterative process, so each step has been broken up into its own sub-task.

Implement Voxel Filter
^^^^^^^^^^^^^^^^^^^^^^

#. Change code:
    
   The first step in most point cloud processing pipelines is the voxel filter. This filter not only helps to downsample your points, but also eliminates any NAN values so that any further filtering or processing is done on real values. See  `PCL Voxel Filter Tutorial <https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html>`_ for more in-depth explanations of the code.
  
   Within ``perception_node.cpp``, find section

   .. code-block:: c++

      /* ========================================
       * Fill Code: VOXEL GRID
       * ========================================*/

   Copy and paste the following code beneath that banner.

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      voxel_filter.setInputCloud(cloud_ptr);
      voxel_filter.setLeafSize(float(0.002), float(0.002), float(0.002));
      voxel_filter.filter(*cloud_voxel_filtered);

#. Update Publisher Within ``perception_node.cpp``, find section

   .. code-block:: c++

      /* ========================================
       * CONVERT POINTCLOUD PCL->ROS
       * PUBLISH CLOUD
       * Fill Code: UPDATE AS NECESSARY
       * ========================================*/

   Replace ``cloud`` with ``*cloud_voxel_filtered`` in the call to ``publishPointCloud(...)`` to publish your new filtered point cloud. Take a look at the function and see how we converted our point cloud to a ROS message in order to publish it.

   .. Note:: For each type of filter we will create a new publisher. It is often useful to view the results of multiple filters at once in Rviz and just toggle different clouds.

#. Compile and source

   .. code-block:: bash

      colcon build
      source install/setup.bash

Viewing Results
"""""""""""""""
#. Run the (currently small) perception pipeline (each line should be run in a separate terminal). Note: In rviz change the global frame to **kinect_link**.

   .. code-block:: bash

      ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world_frame kinect_link

   .. code-block:: bash
      cd ~/perception_ws
      ros2 run lesson_perception pcd_to_pointcloud --ros-args -p filename:=${HOME}/table.pcd -p tf_frame:=kinect_link -p topic:=/kinect/depth_registered/points
      
   .. code-block:: bash
      ros2 run rviz2 rviz2

   .. code-block:: bash
      ros2 launch lesson_perception processing_node.launch.py

#. View results
    
   Within Rviz, add two *PointCloud2* Displays subscribed to the topics "voxel_cluster" and "kinect/depth_registered/points". What you see will be the results of the voxel filter overlaid on the original point cloud.

   .. image:: /_static/cloud_voxel_filtered.png


#. When you are done viewing the results, try changing the voxel filter size from 0.002 to 0.100 and view the results again.  Reset the filter to 0.002 when done.

   * To see the results of this change, use Ctrl+C to kill the perception node, re-build, and re-run the perception node. If you'd like, you can create a new parameter for the voxel filter size to make editing the parameter much easier.

  .. Note:: You do not need to stop any of the other nodes (rviz, pcd_to_pointcloud, etc).

  .. Note:: Changing ColorTransformer to FlatColor will allow you to edit the color of the point clouds. Making them different colors may make differences easier to see as we add more filtered outputs.
   
#. When you are satisfied with the voxel filter, use Ctrl+C to stop the perception node.


Implement Pass-through Filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Change code:

   The next set of useful filtering to get the region of interest, is a series of pass-through filters. These filters crop your point cloud down to a volume of space (if you use x y and z filter). At this point you should apply a series of pass-through filters, one for each the x, y, and z directions. See `PCL Pass-Through Filter Tutorial <https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html>`_ for hints, or use code below.
    
   Within perception_node.cpp, find section

   .. code-block:: c++

      /* ========================================
       * Fill Code: PASSTHROUGH FILTER(S)
       * ========================================*/

   Copy and paste the following code beneath that banner.

   Here we are setting our filters for the x, y, and z axes as well as the limits for each.

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, xyz_filtered_cloud;
      pcl::PassThrough<pcl::PointXYZ> pass_x;
      pass_x.setInputCloud(cloud_voxel_filtered);
      pass_x.setFilterFieldName("x");
      pass_x.setFilterLimits(-1.0, 1.0);
      pass_x.filter(xf_cloud);

      pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
      pcl::PassThrough<pcl::PointXYZ> pass_y;
      pass_y.setInputCloud(xf_cloud_ptr);
      pass_y.setFilterFieldName("y");
      pass_y.setFilterLimits(-1.0, 1.0);
      pass_y.filter(yf_cloud);

      pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
      pcl::PassThrough<pcl::PointXYZ> pass_z;
      pass_z.setInputCloud(yf_cloud_ptr);
      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits(-1.0, 1.0);
      pass_z.filter(xyz_filtered_cloud);

   *You can change the filter limit values to see different results.*

#. Find where the publishers are created and make a new one called ``passthrough_publisher_`` that publishes to the topic "passthrough_cluster".

#. Find where you previously published the last point cloud and now publish your final Passthrough Filter result (``xyz_filtered_cloud``) to your newly made topic. 

#. Re-build and restart your node.

   .. code-block:: bash

      colcon build
      source install/setup.bash
      ros2 launch lesson_perception processing_node.launch.py

#. View results
   Add this new topic ("passthrough_cluster") to your RViz display. Try toggling each point cloud on and off to view the differences. Compare the PointCloud2 displays based on the ``/kinect/depth_registered/points`` (original camera data), ``object_cluster`` (voxel filter), ``passthrough_cluster`` (latest processing step) topics.  Part of the original point cloud has been "clipped" out of the latest processing result.

   .. image:: /_static/zf_cloud.png


  .. Note:: Try modifying the X/Y/Z FilterLimits (e.g. +/- 0.5), re-build, and re-run.  Observe the effects in rviz.  When complete, reset the limits to +/- 1.0.

#. When you are satisfied with the pass-through filter results, press Ctrl+C to kill the node. There is no need to close or kill the other terminals/nodes.

Plane Segmentation
^^^^^^^^^^^^^^^^^^
#. Change code

   This method is one of the most useful for any application where the object is on a flat surface. In order to isolate the objects on a table, you perform a plane fit to the points, which finds the points which comprise the table, and then subtract those points so that you are left with only points corresponding to the object(s) above the table. This is the most complicated PCL method we will be using and it is actually a combination of two: the RANSAC segmentation model, and the extract indices tool. An in depth example can be found on the `PCL Plane Model Segmentation Tutorial <https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html>`_; otherwise you can copy the below code snippet.

   Within perception_node.cpp, find section:

   .. code-block:: c++

      /* ========================================
       * Fill Code: PLANE SEGEMENTATION
       * ========================================*/

   Copy and paste the following code beneath that banner.

   First, we set up a few new point clouds.

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(xyz_filtered_cloud));
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

   Next we create the segmentation object for the planar model and set the parameters.

   .. code-block:: c++

      pcl::SACSegmentation<pcl::PointXYZ> seg;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (200);
      seg.setDistanceThreshold (0.004);

   Now we can segment the largest planar component from the cropped point cloud.

   .. code-block:: c++

      seg.setInputCloud (cropped_cloud);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
          RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.") ;
      }

   Once you have the inliers (points which fit the plane model), then you can extract the indices within the pointcloud data structure of the points which make up the plane.

   .. code-block:: c++

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cropped_cloud);
      extract.setIndices(inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      RCLCPP_INFO(this->get_logger(),
                  "PointCloud2 representing the planar component: '%lu' data points.", cloud_plane->points.size());

   Then of course you can subtract or filter out these points from the cloud to get only points above the plane.

   .. code-block:: c++

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);

#. Find where the publishers are created and make a new one called ``plane_publisher_`` that publishes to the topic "plane_cluster".

#. Find where you previously published the last point cloud and now publish your plane-fit outliers result (``*cloud_f``) to your newly made topic. 

#. Compile and run, as in previous steps.

#. Evaluate Results

   Within Rviz, compare PointCloud2 displays based on the ``/kinect/depth_registered/points`` (original camera data) and your new topic.  Only points lying above the table plane remain in the latest processing result.

   .. image:: /_static/cloud_f.png

#. When you are done viewing the results you can go back and change the"setMaxIterations" and "setDistanceThreshold" values to control how tightly the plane-fit classifies data as inliers/outliers, and view the results again.  Try using values of ``MaxIterations=100`` and ``DistanceThreshold=0.010``

#. When you are satisfied with the plane segmentation results, use Ctrl+C to kill the node. There is no need to close or kill the other terminals/nodes.


Euclidean Cluster Extraction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#. Change code

   This method is useful for any application where there are multiple objects. This is also a complicated PCL method. A more in-depth explanation of the code can be found at `PCL Euclidean Cluster Extraction Tutorial <https://pcl.readthedocs.io/en/latest/cluster_extraction.html>`_.

   Within perception_node.cpp, find section

   .. code-block:: c++

      /* ========================================
       * Fill Code: EUCLIDEAN CLUSTER EXTRACTION 
       * ========================================*/

   Copy and paste the following code beneath the banner.

   First we create a KdTree object to use as the search method of our cluster extraction.

   .. code-block:: c++

      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      *cloud_filtered = *cloud_f;
      tree->setInputCloud (cloud_filtered);

   Next we create a vector of PointIndices that will save the indices of our detected clusters. Here, we will also set the tolerance, minimum cluster size, and maximum cluster size for our search.

   .. code-block:: c++

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance (0.01);
      ec.setMinClusterSize (1);
      ec.setMaxClusterSize (10000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud_filtered);
      ec.extract (cluster_indices);

      std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> pc2_clusters;
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;

   Now we can begin looping through our extracted clusters and separate them out of our vector of PointIndices in order to create a new point clouds.

   .. code-block:: c++

      int j = 0;
      for (const auto& cluster : cluster_indices)
      {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

          for (const auto& idx : cluster.indices) {
              cloud_cluster->points.push_back((*cloud_filtered)[idx]);
          }

          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;
          RCLCPP_INFO(this->get_logger(), "Cluster has '%lu' points", cloud_cluster->points.size());
          clusters.push_back(cloud_cluster);
          sensor_msgs::msg::PointCloud2::SharedPtr tempROSMsg(new sensor_msgs::msg::PointCloud2);
          pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
          pc2_clusters.push_back(tempROSMsg);

          j++;

      }
      RCLCPP_INFO(this->get_logger(), "Largest cluster has '%lu' points", clusters.at(0)->points.size());

#. Find where the publishers are created and make a new one called ``euclidean_publisher_`` that publishes to the topic "euclidean_cluster".

#. Find where you previously published the last point cloud and now publish your largest cluster (``*(clusters.at(0))``) to your newly made topic. 

#. Compile and run, as in previous steps.

#. View results in rviz.  Experiment with ``setClusterTolerance``, ``setMinClusterSize``, and ``setMaxClusterSize`` parameters, observing their effects in rviz.

   .. image:: /_static/clusters_at0.png


#. When you are satisfied with the cluster extraction results, use Ctrl+C to kill the node. There is no need to close or kill the other terminals/nodes.

Create a CropBox Filter (Optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Change code

   This method is similar to the pass-through filter from Sub-Task 2, but instead of using three pass-through filters in series, you can use one CropBox filter. Documentation on the CropBox filter and necessary header file can be found `here <https://pointclouds.org/documentation/classpcl_1_1_crop_box.html>`_.

   Within perception_node.cpp, find section

   .. code-block:: c++

      /* ========================================
       * Fill Code: CROPBOX (OPTIONAL)
       * ========================================*/

   This CropBox filter should replace your passthrough filters so you may delete or comment out the passthrough filters. There is no PCL tutorial to guide you, only the PCL documentation at the link above. The general setup will be the same (set the output, declare instance of filter, set input, set parameters, and filter).

   Set the output cloud:

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ> xyz_filtered_cloud;

   Declare instance of filter:

   .. code-block:: c++

      pcl::CropBox<pcl::PointXYZ> crop;

   Set input:

   .. code-block:: c++

      crop.setInputCloud(cloud_voxel_filtered);

   Set parameters - looking at documentation, CropBox takes an Eigen Vector4f as inputs for max and min values:

   .. code-block:: c++

      Eigen::Vector4f min_point = Eigen::Vector4f(-1.0, -1.0, -1.0, 0);
      Eigen::Vector4f max_point = Eigen::Vector4f(1.0, 1.0, 1.0, 0);
      crop.setMin(min_point);
      crop.setMax(max_point);

   Filter:

   .. code-block:: c++

      crop.filter(xyz_filtered_cloud);

   If you delete or comment out the passthrough filters and have already written the plane segmentation code, then make sure you update the name of the cloud you are passing into the plane segmentation. Replace zf_cloud with xyz_filtered_cloud:

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(xyz_filtered_cloud));


#. Find where you publish your filtered clouds and replace the cloud for the passthrough filter with your new filtered results (``xyz_filtered_cloud``).

#. Compile and run, as in previous steps

    The following image of the CropBox filter in use will closely resemble the Plane Segmentation filter image.

   .. image:: /_static/xyz_filtered_cloud.png


Create a Statistical Outlier Removal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Change code

   This method does not necessarily add complexity or information to our end result, but it is often useful. A tutorial can be found `here <https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html>`_.

   Within perception_node.cpp, find section

   .. code-block:: c++

      /* ========================================
       * Fill Code: STATISTICAL OUTLIER REMOVAL
       * ========================================*/

   The general setup will be the same (set the output, declare instance of filter, set input, set parameters, and filter).

   Set the output cloud:

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_ptr= clusters.at(0);
      pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

   Declare instance of filter:

   .. code-block:: c++

      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

   Set input:

   .. code-block:: c++

      sor.setInputCloud (cluster_cloud_ptr);

   Set parameters - looking at documentation, S.O.R. uses the number of neighbors to inspect and the standard-deviation threshold to use for outlier rejection:

   .. code-block:: c++

      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);

   Filter:

   .. code-block:: c++

      sor.filter (*sor_cloud_filtered);

#. Find where the publishers are created and make a new one called ``stat_publisher_`` that publishes to the topic "stat_cluster".

#. Find where you previously published the last point cloud and now publish your new filtered results (``*sor_cloud_filtered``) to your newly made topic. 

#. Compile and run, as in previous steps

   .. image:: /_static/sor_cloud_filtered.png


Create a Broadcast Transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

While this is not a filter method, it demonstrates how to publish the results of a processing pipeline for other nodes to use.  Often, the goal of a processing pipeline is to generate a measurement, location, or some other message for other nodes to use.  This sub-task broadcasts a TF transform to define the location of the largest box on the table.  This transform could be used by other nodes to identify the position/orientation of the box for grasping.

#. Change/Insert code

   Within perception_node.cpp, find section

   .. code-block:: c++

      /* ========================================
       * BROADCAST TRANSFORM 
       * ========================================*/

   You can follow along with the `ROS Tutorial <https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html>`_. 

   You'll notice that we have already created a ``tf2_ros::Buffer`` and a ``tf2_ros::TransformListener`` for you in the initialization of our class. Create a transform using these:

   .. code-block:: c++

      std::unique_ptr<tf2_ros::TransformBroadcaster> br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      geometry_msgs::msg::TransformStamped part_transform;

      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      part_transform.transform.rotation.x = q.x();
      part_transform.transform.rotation.y = q.y();
      part_transform.transform.rotation.z = q.z();
      part_transform.transform.rotation.w = q.w();

      //Here x,y, and z should be calculated based on the PointCloud2 filtering results
      part_transform.transform.translation.x = sor_cloud_filtered->at(1).x;
      part_transform.transform.translation.y = sor_cloud_filtered->at(1).y;
      part_transform.transform.translation.z = sor_cloud_filtered->at(1).z;
      part_transform.header.stamp = this->get_clock()->now();
      part_transform.header.frame_id = world_frame;
      part_transform.child_frame_id = "part";


   Remember that when you set the origin or set the rpy, this is where you should use the results from all the filters you've applied. At this point the origin is set arbitrarily to the first point within. Broadcast that transform:

   .. code-block:: c++

      br->sendTransform(part_transform);

#. Compile and Run as usual.  In this case, add a TF display to Rviz and observe the new "part" transform located at the top of the box.

Create a Polygonal Segmentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When using sensor data for collision detection, it is sometimes necessary to exclude "known" objects from the scene to avoid interference from these objects.  MoveIt! contains methods for masking out a robot's own geometry as a "Self Collision" filtering feature.  This example shows how to do something similar using PCL's Polygonal Segmentation filtering.

#. Change code

   This method is similar to the plane segmentation from Sub-Task 3, but instead of segmenting out a plane, you can segment and remove a prism. Documentation on the PCL Polygonal Segmentation can be found `here <https://pcl.readthedocs.io/projects/tutorials/en/latest/hull_2d.html>`_. The goal in this sub-task is to remove the points that correspond to a known object (e.g. the box we detected earlier). This particular filter is applied to the entire point cloud (original sensor data), but only after we've already completed the processing steps to identify the position/orientation of the box.

   Within perception_node.cpp find section

   .. code-block:: c++

      /* ========================================
       * Fill Code: POLYGONAL SEGMENTATION 
       * ========================================*/

   Set the input cloud:

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud));
      pcl::PointCloud<pcl::PointXYZ>::Ptr prism_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pick_surface_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

   Declare instance of filter:

   .. code-block:: c++

      pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;

   Set extraction indices:

   .. code-block:: c++

      pcl::ExtractIndices<pcl::PointXYZ> extract_ind;

   Set input and output:

   .. code-block:: c++

      prism.setInputCloud(sensor_cloud_ptr);
      pcl::PointIndices::Ptr pt_inliers (new pcl::PointIndices());

   Set parameters - looking at documentation, ExtractPolygonalPrismData uses a pointcloud defining the polygon vertices as its input.

   .. code-block:: c++

      // create prism surface
      double box_length=0.25;
      double box_width=0.25;
      pick_surface_cloud_ptr->width = 5;
      pick_surface_cloud_ptr->height = 1;
      pick_surface_cloud_ptr->points.resize(5);

      pick_surface_cloud_ptr->points[0].x = 0.5f*box_length;
      pick_surface_cloud_ptr->points[0].y = 0.5f*box_width;
      pick_surface_cloud_ptr->points[0].z = 0;

      pick_surface_cloud_ptr->points[1].x = -0.5f*box_length;
      pick_surface_cloud_ptr->points[1].y = 0.5f*box_width;
      pick_surface_cloud_ptr->points[1].z = 0;

      pick_surface_cloud_ptr->points[2].x = -0.5f*box_length;
      pick_surface_cloud_ptr->points[2].y = -0.5f*box_width;
      pick_surface_cloud_ptr->points[2].z = 0;

      pick_surface_cloud_ptr->points[3].x = 0.5f*box_length;
      pick_surface_cloud_ptr->points[3].y = -0.5f*box_width;
      pick_surface_cloud_ptr->points[3].z = 0;

      pick_surface_cloud_ptr->points[4].x = 0.5f*box_length;
      pick_surface_cloud_ptr->points[4].y = 0.5f*box_width;
      pick_surface_cloud_ptr->points[4].z = 0;

      Eigen::Affine3d eigen3d = tf2::transformToEigen(part_transform);
      pcl::transformPointCloud(*pick_surface_cloud_ptr,*pick_surface_cloud_ptr,Eigen::Affine3f(eigen3d));

      prism.setInputPlanarHull( pick_surface_cloud_ptr);
      prism.setHeightLimits(-10,10);

   Segment:

   .. code-block:: c++

      prism.segment(*pt_inliers);

   Remember that after you use the segmentation algorithme that you either want to include or exclude the segmented points using an index extraction.

   Set input:

   .. code-block:: c++

      extract_ind.setInputCloud(sensor_cloud_ptr);
      extract_ind.setIndices(pt_inliers);

   This time, we invert the index extraction, so that we remove points inside the filter and keep points outside the filter.

   .. code-block:: c++

      extract_ind.setNegative(true);

   Filter:

   .. code-block:: c++

      extract_ind.filter(*prism_filtered_cloud);

#. Find where the publishers are created and make a new one called ``polygon_publisher_`` that publishes to the topic "polygon_cluster".

#. Find where you previously published the last point cloud and now publish your new filtered results (``*prism_filtered_cloud``) to your newly made topic. 

#. Compile and run as before.

    .. image:: /_static/prism_filtered_cloud.png

   .. Note:: Notice that the target box has been removed from the point cloud display.

Using Parameters
^^^^^^^^^^^^^^^^^^^

While this is not a filter method, it is useful when using PCL or other perception methods because of the number of parameters used in the different methods.

#. Change/Insert code

   For this exercise, we will be declaring the parameters from within ``processing_node.launch.py``.

   In ``perception_node.cpp``, find section

   .. code-block:: c++

      /*
       * SET UP PARAMETERS (COULD TO BE INPUT FROM LAUNCH FILE/TERMINAL)
       */

   Notice our use of ``rclcpp::NodeOptions()`` at the start of our class declaration. In our node we use ``get_parameter_or(...)`` to get each parameter or give it a default value if no value has been assigned yet. This way of calling a parameter will return a ``rclcpp::Parameter`` object (unlike in ROS 1 that returned an instance of the parameter's type, i.e. a string). You will then need to retrieve the value of the parameter using ``.value()`` or a more specific ``.as_string()``, ``.as_int()``, ``.as_double()``, etc.

   Take a look at the 3 parameters we have already created for you (cloud_topic, world_frame, camera_frame) and how we have declared them in both our node and launch file. 
   Try creating some new parameters to replace some of our hard-coded values in our filters and test them out. Below is an example of some of the parameters you could have set.

   .. code-block:: yaml

      world_frame: "kinect_link"
      camera_frame: "kinect_link"
      cloud_topic: "kinect/depth_registered/points"
      voxel_leaf_size: 0.02
      x_filter_min: -2.5
      x_filter_max: 2.5
      y_filter_min: -2.5
      y_filter_max: 2.5
      z_filter_min: -2.5
      z_filter_max: 1.0
      plane_max_iter: 50
      plane_dist_thresh: 0.05
      cluster_tol: 0.01
      cluster_min_size: 100
      cluster_max_size: 50000


   You will need to edit both ``perception_node.cpp`` and ``processing_node.launch.py``.

   When you are satisfied with the results, go to each terminal and *CTRL-C*.

   You're all done! So it's best to make sure everything is wrapped up and closed.
