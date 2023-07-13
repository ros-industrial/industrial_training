Building a Simple PCL Interface for Python
==========================================

In this exercise, we will fill in the appropriate pieces of code to build a perception pipeline. The end goal will be to create point cloud filtering operations to demonstrate functionality between ROS and python.


Prepare New Workspace:
----------------------

We will create a new ROS 2 workspace, since this exercise does not overlap with the previous ScanNPlan exercises.

#. Disable automatic sourcing of your previous workspace (if you have any) in your ``.bashrc``.

#. Copy the template workspace layout and files:

   .. code-block:: bash

            mkdir ~/python_pcl_ws
            cp -r ~/industrial_training/exercises/5.3/template_ws/ros2/src ~/python_pcl_ws
            cd ~/python_pcl_ws/

#. Initialize and Build this new workspace

   .. code-block:: bash

            source /opt/ros/foxy/setup.bash
            colcon build


#. Source the workspace

   .. code-block:: bash

            source ~/python_pcl_ws/install/setup.bash

#. Download the PointCloud file and place the file in your home directory :

   .. code-block:: bash
   
            cp -r ~/industrial_training/exercises/4.2/table.pcd ~


Intro (Review Existing Code)
----------------------------

Most of the infrastructure for a ROS 2 node has already been completed for you; the focus of this exercise is the perception algorithms/pipeline and the use of a pure Python ROS package. There are two packages provided for you: ``py_perception`` and ``filter_call``. 

At this time we will explore the source code that has been provided in the ``py_perception_node.cpp`` file. This tutorial has been modified from training `Exercise 5.1 Building a Perception Pipeline <http://ros-industrial.github.io/industrial_training/_source/session5/Building-a-Perception-Pipeline.html>`__ and as such the C++ code has already been set up. Open the ``perception_node.cpp`` file and review the filtering functions.

.. Note:: For an extra challenge, after completing exercise 5.1, try adding the missing filters into our new ``py_perception_node`` here. 

Using a Python Package
^^^^^^^^^^^^^^^^^^^^^^

Now that we have converted several filters to C++ functions, we are ready to call it from a Python node.

Take a look at the ``filter_call`` package and note the differences in structure between a pure Python package and a C++ package. 

We will not be including ‘perception_msgs’ as a dependency as we will not be creating custom messages in this course. If you wish for a more in depth explanation including how to implement custom messages, `MIT has a resource <http://duckietown.mit.edu/media/pdfs/1rpRisFoCYUm0XT78j-nAYidlh-cDtLCdEbIaBCnx9ew.pdf>`__ on the steps taken.

Notice that instead of a ``CMakeLists.txt`` file we have ``setup.cfg`` and ``setup.py``. The ``setup.py`` file functions similarly to a ``CMakeLists.txt`` and ``setup.cfg`` tells the package where our scripts will be installed. In order for this folder to be accessed by any other python script, the ``__init__.py`` file must exist. In this example, it is located at ``filter_call/filter_call/__init__.py``. ``filter_call/filter_call`` is also where our scripts will live.


Publishing the Point Cloud
^^^^^^^^^^^^^^^^^^^^^^^^^^

As mentioned above, we are creating a ROS 2 C++ node to filter the point cloud when requested by a Python node running a service request for each filtering operation, resulting in a new, aggregated point cloud.  Let’s start by modifying our C++ code to publish in a manner supportive to Python. 

Implement a Voxel Filter
^^^^^^^^^^^^^^^^^^^^^^^^

#. In ``py_perception_node.cpp``, take notice of the function called ``filterCallBack``. This function will be the entry point for all service calls made by the Python client in order to run point cloud filtering operations.

   .. code-block:: c++

      void filterCallback(py_perception::srv::FilterCloud::Request::SharedPtr request,
                              py_perception::srv::FilterCloud::Response::SharedPtr response)
          {
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
              pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
              if (request->pcdfilename.empty())
              {
                  pcl::fromROSMsg(request->input_cloud, *cloud);
                  RCLCPP_INFO(this->get_logger(), "cloud size: '%lu'", cloud->size());
                  if (cloud->empty())
                  {
                      RCLCPP_ERROR(this->get_logger(), "input cloud empty");
                      response->success = false;
                      return;
                  }
              }
              else
              {
                  pcl::io::loadPCDFile(request->pcdfilename, *cloud);
              }
              switch (request->operation)
              {
                  case py_perception::srv::FilterCloud::Request::VOXELGRID :
                  {
                      filtered_cloud = voxelGrid(cloud, 0.01);
                      break;
                  }
                  default :
                  {
                      RCLCPP_ERROR(this->get_logger(), "no point cloud found");
                      response->success = false;
                      return;
                  }
              }
              /*
               * SETUP RESPONSE
               */
              pcl::toROSMsg(*filtered_cloud, response->output_cloud);
              response->output_cloud.header = request->input_cloud.header;
              response->output_cloud.header.frame_id = "kinect_link";
              response->success = true;
          }


#. Now that we have the framework for the filtering on the server side, let's start setting up the client side. Find and open the script ``filter_call.py`` in your ``filter_call`` package.

#. Examine the provided code and functions in the script. Take note of some similarities and differences between how the Python node is set up versus a C++ node. 


#. Call the service to apply a Voxel Grid filter. Find the function for applying a voxel grid filter and insert the following code below the banner

   .. code-block:: python
   
        # =======================
        # VOXEL GRID FILTER
        # =======================
        
   #. We first create a service request of type FilterCloud and populate it with the necessary information:

      .. code-block:: python

           req = FilterCloud.Request()
           req.pcdfilename = self.pcdfilename
           req.operation = py_perception.srv.FilterCloud.Request.VOXELGRID
           req.input_cloud = PointCloud2()

           # ERROR HANDLING
           if req.pcdfilename == '':
               self.get_logger().error('No file parameter found')
               return

   #. Next we can send a request to the server node and wait for a response:

      .. code-block:: python

           future = self.client.call_async(req)
           rclpy.spin_until_future_complete(self, future)
           res_voxel = future.result()
           if not res_voxel.success:
               self.get_logger().error('Unsuccessful voxel grid filter operation')
               return

   #. Finally, we publish our new filtered point cloud:

      .. code-block:: python

           self.voxel_pub.publish(res_voxel.output_cloud)
           self.last_cloud = res_voxel
           self.get_logger().info("published: voxel grid filter response")


#. Before running our new node, we need to make the Python file executable. Open ``setup.py`` and modify ``entry_points`` to read

   .. code-block:: python

         entry_points={
           'console_scripts': [
               'filter_call = filter_call.filter_call:main'
           ],

#. Re-build and re-source your workspace.


Viewing Results
^^^^^^^^^^^^^^^

#. In your terminal, source a new terminal and run the C++ filter service node

   .. code-block:: bash

            ros2 run py_perception perception_node

#. Source a new terminal and run the ``tf2_ros`` package to publish a static coordinate transform from the child frame to the world frame

   .. code-block:: bash
   
            ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world_frame kinect_link

#. Source a new terminal and run the Python service client node. Note your file path may be different.

   .. code-block:: bash

            ros2 run filter_call filter_call --ros-args -p pcdfilename:=~/table.pcd

#. Source a new terminal and run Rviz

   .. code-block:: bash

            ros2 run rviz2 rviz2

#. Add a new PointCloud2 in Rviz

#. In global options, change the fixed frame to **kinect_link** or **world_frame**, and in the PointCloud 2, select your topic to be '/perception_voxelGrid'

#. You should be able to see your filtered point cloud in Rviz.


Implement Pass-Through Filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. In ``py_perception_node.cpp`` in the ``py_perception`` package, update the switch to also take a ``PASSTHROUGH`` option like below:

   .. code-block:: bash

        switch (request->operation)
        {
            case py_perception::srv::FilterCloud::Request::VOXELGRID :
            {
                filtered_cloud = voxelGrid(cloud, 0.01);
                break;
            }
            case py_perception::srv::FilterCloud::Request::PASSTHROUGH :
            {
                filtered_cloud = passThrough(cloud);
                break;
            }
            default :
            {
                RCLCPP_ERROR(this->get_logger(), "no point cloud found");
                response->success = false;
                return;
            }
        }

#. Save and build


   **Edit the Python Code**


#. Open the python node and copy paste the following code inside the ``passthrough_filter`` function under the banner.  Keep care to maintain indents:

   .. code-block:: python

        # =======================
        # PASSTHROUGH FILTER
        # =======================

   #. Again, we need to first create and populate our FilterCloud service request:

      .. code-block:: python

            req = FilterCloud.Request()
            req.pcdfilename = ''
            req.operation = py_perception.srv.FilterCloud.Request.PASSTHROUGH
            req.input_cloud = self.last_cloud.output_cloud

   #. Next we call the server and wait for a response:

      .. code-block:: python

           future = self.client.call_async(req)
           rclpy.spin_until_future_complete(self, future)
           res_pass = future.result()
           if not res_pass.success:
               self.get_logger().error('Unsuccessful passthrough filter operation')
               return

   #. Finally we publish our result:

      .. code-block:: python

           self.pass_pub.publish(res_pass.output_cloud)
           self.last_cloud = res_pass
           self.get_logger().info("published: passthrough filter response")

#. Save and run from the terminal, repeating steps outlined for the voxel filter.

   Within Rviz, compare PointCloud2 displays based on the the previous voxel grid filter and your new point cloud.

   When you are satisfied with the pass-through filter results, press Ctrl+C to kill the node. There is no need to close or kill the other terminals/nodes.

.. Note:: Did you forget to create a new publisher for the passthrough filter? And did you remember to call ``pasthrough_filter()``? Try taking a look at where we create ``voxel_pub`` and call ``voxel_filter()`` for help.


Plane Segmentation
^^^^^^^^^^^^^^^^^^

This method is one of the most useful for any application where the object is on a flat surface. In order to isolate the objects on a table, you perform a plane fit to the points, which finds the points which comprise the table, and then subtract those points so that you are left with only points corresponding to the object(s) above the table. This is the most complicated PCL method we will be using and it is actually a combination of two: the RANSAC segmentation model, and the extract indices tool. An in depth example can be found on the `PCL Plane Model Segmentation Tutorial <https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html>`__; otherwise you can copy the below code snippet.


#. In ``py_perception_node.cpp``, update the switch statement in ``filterCallback`` to also take a ``PLANESEGMENTATION`` option:

   .. code-block:: c++

            case py_perception::srv::FilterCloud::Request::PLANESEGMENTATION :
            {
                filtered_cloud = planeSegmentation(cloud);
                break;
            }



#. Save and build

   **Edit the Python Code**

#. Open the python node and copy paste the following code inside the ``plane_segmentation`` function under the banner.  Keep care to maintain indents:

   .. code-block:: python

        # =======================
        # PLANE SEGMENTATION
        # =======================

   #. Again, we need to first create and populate our FilterCloud service request:

      .. code-block:: python

           req = FilterCloud.Request()
           req.pcdfilename = ''
           req.operation = py_perception.srv.FilterCloud.Request.PLANESEGMENTATION
           req.input_cloud = self.last_cloud.output_cloud

   #. Next we call the server and wait for a response:

      .. code-block:: python

           future = self.client.call_async(req)
           rclpy.spin_until_future_complete(self, future)
           res_seg = future.result()
           if not res_seg.success:
               self.get_logger().error('Unsuccessful plane segmentation operation')
               return

   #. Finally we publish our result:

      .. code-block:: python

           self.plane_pub.publish(res_seg.output_cloud)
           self.last_cloud = res_seg
           self.get_logger().info("published: plane segmentation filter response")


#. Save and run from the terminal, repeating steps outlined above.

   Within Rviz, compare PointCloud2 displays based on your previous filters and your new one.

   #. When you are done viewing the results you can go back and change the ``setMaxIterations`` and ``setDistanceThreshold`` parameter values to control how tightly the plane-fit classifies data as inliers/outliers, and view the results again. Try using values of ``maxIterations=100`` and ``distThreshold=0.010``

   #. When you are satisfied with the plane segmentation results, use Ctrl+C to kill the node. There is no need to close or kill the other terminals/nodes.


Euclidian Cluster Extraction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This method is useful for any application where there are multiple objects. This is also a complicated PCL method. An in depth example can be found on the `PCL Euclidean Cluster Extration Tutorial <https://pcl.readthedocs.io/en/latest/cluster_extraction.html>`_.


#. In ``py_perception_node.cpp``, update the switch statement in ``filterCallback`` to to also take a ``CLUSTEREXTRACTION`` option:

   .. code-block:: c++

            case py_perception::srv::FilterCloud::Request::CLUSTEREXTRACTION :
            {
                filtered_cloud = clusterExtraction(cloud).at(0);
                break;
            }


#. Save and build


   **Edit the Python Code**


#. Open the python node and copy paste the following code inside the ``plane_segmentation`` function under the banner.  Keep care to maintain indents:

   .. code-block:: python

        # =======================
        # CLUSTER EXTRACTION
        # =======================

   #. Again, we need to first create and populate our FilterCloud service request:

      .. code-block:: python

           req = FilterCloud.Request()
           req.pcdfilename = ''
           req.operation = py_perception.srv.FilterCloud.Request.CLUSTEREXTRACTION
           req.input_cloud = self.last_cloud.output_cloud

   #. Next we call the server and wait for a response:

      .. code-block:: python

           future = self.client.call_async(req)
           rclpy.spin_until_future_complete(self, future)
           res_cluster = future.result()
           if not res_cluster.success:
               self.get_logger().error('Unsuccessful cluster extraction operation')
               return

   #. Finally we publish our result:

      .. code-block:: python

           self.cluster_pub.publish(res_cluster.output_cloud)
           self.last_cloud = res_cluster
           self.get_logger().info("published: cluster extraction filter response")


#. Save and run from the terminal, repeating steps outlined above.

   Within Rviz, compare PointCloud2 displays based on your previous filters and your new one.

   #. When you are satisfied with the cluster extraction results, use Ctrl+C to kill the node. If you are done experimenting with this tutorial, you can kill the nodes running in the other terminals.


Future Study
^^^^^^^^^^^^

For an extra challenge, you can convert the remaining filters from Exercise 5.1 into callable functions and add options to call them in your service and Python node.

Additionally, the Python code was repeated for each filtering intance for simplicity. Another option is to create a loop or function to replace the repeated chunks of code. Feel free to play around with these options to better refine your code.
