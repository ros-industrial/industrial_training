Building a Simple PCL Interface for Python
==========================================

In this exercise, we will fill in the appropriate pieces of code to build a perception pipeline. The end goal will be to create point cloud filtering operations to demonstrate functionality between ROS and python.


Prepare New Workspace:
----------------------

We will create a new catkin workspace, since this exercise does not overlap with the previous ScanNPlan exercises.

#. Disable automatic sourcing of your previous catkin workspace:

   #. ``gedit ~/.bashrc``

   #. Comment out the line of your ``.bashrc`` file which sources the previous workspace

      .. code-block:: bash

               source /opt/ros/melodic/setup.bash
               # source ~/catkin_ws/devel/setup.bash

#. Copy the template workspace layout and files:

   .. code-block:: bash

            cp -r ~/industrial_training/exercises/python-pcl_ws ~
            cd ~/python-pcl_ws/

#. Initialize and Build this new workspace

   .. code-block:: bash

            catkin init
            catkin build


#. Source the workspace

   .. code-block:: bash

            source ~/python-pcl_ws/devel/setup.bash

#. Download the PointCloud file and place the file in your workspace's **src** directory :

   .. code-block:: bash
   
      cp -r ~/industrial_training/exercises/4.2/table.pcd src/


Intro (Review Existing Code)
----------------------------

Most of the infrastructure for a ROS node has already been completed for you; the focus of this exercise is the perception algorithms/pipeline. The ``CMakelists.txt`` and ``package.xml`` are complete and a source file has been provided. At this time we will explore the source code that has been provided in the ``py_perception_node.cpp`` file. This tutorial has been modified from training `Exercise 5.1 Building a Perception Pipeline <http://ros-industrial.github.io/industrial_training/_source/session5/Building-a-Perception-Pipeline.html>`__ and as such the C++ code has already been set up. Open the ``perception_node.cpp`` file and review the filtering functions.

Create a Python Package
^^^^^^^^^^^^^^^^^^^^^^^

Now that we have converted several filters to C++ functions, we are ready to call it from a Python node.

#. In the terminal, change the directory to your src folder. Create a new package inside your python-pcl_ws:

   .. code-block:: bash

            cd ~/python-pcl_ws/src/
            catkin create pkg test_pkg_python --catkin-deps rospy

#. Check that your package was created:

   .. code-block:: bash

            ls 

We will not be including ‘perception_msgs’ as a dependency as we will not be creating custom messages in this course. If you wish for a more in depth explanation including how to implement customer messages, here is a good `MIT resource <http://duckietown.mit.edu/media/pdfs/1rpRisFoCYUm0XT78j-nAYidlh-cDtLCdEbIaBCnx9ew.pdf>`__ on the steps taken.


#. Open ``CMakeLists.txt``. Uncomment line 19 or wherever you find **# catkin_python_setup()** and save.

   .. code-block:: bash

            catkin_python_setup()

Creating setup.py
^^^^^^^^^^^^^^^^^

The ``setup.py`` file makes your python module available to the entire workspace and subsequent packages.  By default, this isn’t created by the ``catkin_create_pkg`` command.

#. In your terminal type

   .. code-block:: bash

            gedit filter_call/setup.py

#. Copy and paste the following to the ``setup.py`` file

   .. code-block:: python

            ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
            from distutils.core import setup
            from catkin_pkg.python_setup import generate_distutils_setup
            # fetch values from package.xml
            setup_args = generate_distutils_setup(
            packages=[''],
            package_dir={'': 'include'},
            )
            setup(**setup_args)


   Change ``packages = [ . . . ],`` to your list of strings of the name of the folders inside your *include* folder.  By convention, this will be the same name as the package, or ``filter_call`` . The configures ``filter_call/include/filter_call`` as a python module available to the whole workspace.

#. Save and close the file.

    In order for this folder to be accessed by any other python script, the ``__init__.py`` file must exist.

#. Create one in the terminal by typing:

   .. code-block:: bash

            touch filter_call/include/filter_call/__init__.py

Publishing the Point Cloud
^^^^^^^^^^^^^^^^^^^^^^^^^^

As iterated before, we are creating a ROS C++ node to filter the point cloud when requested by a Python node running a service request for each filtering operation, resulting in a new, aggregated point cloud.  Let’s start with modifying our C++ code to publish in a manner supportive to python. Remember, the C++ code is already done so all you need to do is write your python script and view the results in Rviz.

Implement a Voxel Filter
^^^^^^^^^^^^^^^^^^^^^^^^

#. In ``py_perception_node.cpp``, take notice of the function called ``filterCallBack`` (around line 170). This function will be the entry point for all service calls made by the Python client in order to run point cloud filtering operations.

   .. code-block:: c++

        bool filterCallback(lesson_perception::FilterCloud::Request& request,
                            lesson_perception::FilterCloud::Response& response)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
          pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

          if (request.pcdfilename.empty())
          {
            pcl::fromROSMsg(request.input_cloud, *cloud);
            ROS_INFO_STREAM("cloud size: " << cloud->size());
          }
          else
          {
            pcl::io::loadPCDFile(request.pcdfilename, *cloud);
          }

          if (cloud->empty())
          {
            ROS_ERROR("input cloud empty");
            response.success = false;
            return false;
          }

          switch (request.operation)
          {

            case lesson_perception::FilterCloud::Request::VOXELGRID :
            {
              filtered_cloud = voxelGrid(cloud, 0.01);
              break;
            }
            default :
            {
              ROS_ERROR("No valid request found");
              return false;
            }

           }

        /*
         * SETUP RESPONSE
         */
          pcl::toROSMsg(*filtered_cloud, response.output_cloud);
          response.output_cloud.header=request.input_cloud.header;
          response.output_cloud.header.frame_id="kinect_link";
          response.success = true;
          return true;
        }


#. Within ``main``, take notice of the lines starting at 244, this is where we load the parameters used by the various filters. 

   .. code-block:: c++

            priv_nh_.param<double>("leaf_size", leaf_size_, 0.0f); 
   Build the package and go into the **filter_call** package now

#. Now that we have the framework for the filtering, open your terminal. Make sure you are in the ``filter_call`` directory. Create a ``scripts`` folder.

   .. code-block:: bash

            mkdir scripts

#. Copy and paste the following code at the top of ``filter_call.py`` to import necessary libraries:

   .. code-block:: python

            #!/usr/bin/env python

            import rospy
            import lesson_perception.srv
            from sensor_msgs.msg import PointCloud2

#. We will create an ``if`` statement that contains the ``main`` function that is called when the node is run from the command line. Paste the following after the import statements:

   .. code-block:: python

        if __name__ == '__main__':
            try:
               rospy.init_node('filter_cloud', anonymous=True)
               rospy.wait_for_service('filter_cloud')
               
               rospy.spin()
            except Exception as e:
                print("Service call failed: %s" % str(e))
   #. The ``rospy.init_node`` function initializes the node and gives it a name
   #. The ``rospy.wait_for_service`` waits for the ``filter_cloud`` service.
   #. The ``rospy.spin`` is the Python counterpart of the ``roscpp::spin()`` function in C++.


#. Call the service to apply a Voxel Grid filter. Copy and paste the following inside the ``try`` block in the line following the ``rospy.wait_for_service`` function:

   .. code-block:: python
   
        # =======================
        # VOXEL GRID FILTER
        # =======================
        
        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = rospy.get_param('~pcdfilename', '')
        req.operation = lesson_perception.srv.FilterCloudRequest.VOXELGRID

        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = PointCloud2()

        # ERROR HANDLING
        if req.pcdfilename == '':
            raise Exception('No file parameter found')

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_voxel = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful voxel grid filter operation')

        # PUBLISH VOXEL FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_voxelGrid', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_voxel.output_cloud)
        print("published: voxel grid filter response")


#. We need to make the Python file executable. In your terminal:

   .. code-block:: bash

            sudo chmod +x filter_call/scripts/filter_call.py

Viewing Results
^^^^^^^^^^^^^^^

#. In your terminal, run

   .. code-block:: bash

            roscore

#. Source a new terminal and run the C++ filter service node

   .. code-block:: bash

            rosrun lesson_perception py_perception_node

#. Source a new terminal and run the Python service client node. Note your file path may be different.

   .. code-block:: bash

            rosrun filter_call filter_call.py _pcdfilename:=`d`/src/table.pcd

#. Source a new terminal and run the ``tf2_ros`` package to publish a static coordinate transform from the child frame to the world frame

   .. code-block:: bash
   
            rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world_frame kinect_link

#. Source a new terminal and run Rviz

   .. code-block:: bash

            rosrun rviz rviz

#. Add a new PointCloud2 in Rviz

#. In global options, change the fixed frame to **kinect_link** or **world_frame**, and in the PointCloud 2, select your topic to be '/perception_voxelGrid'

   .. Note::

        You may need to uncheck and recheck the PointCloud2.

Implement Pass-Through Filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. In ``py_perception_node.cpp`` in the ``lesson_perception`` package, update the switch to look as shown below:

   .. code-block:: bash

        switch (request.operation)
        {

          case lesson_perception::FilterCloud::Request::VOXELGRID :
          {
            filtered_cloud = voxelGrid(cloud, 0.01);
            break;
          }
          case lesson_perception::FilterCloud::Request::PASSTHROUGH :
          {
            filtered_cloud = passThrough(cloud);
            break;
          }
          default :
          {
            ROS_ERROR("No valid request found");
            return false;
          }

        }

#. Save and build


   **Edit the Python Code**


#. Open the python node and copy paste the following code after the voxel grid, before the ``rospy.spin()``.  Keep care to maintain indents:

   .. code-block:: python

        # =======================
        # PASSTHROUGH FILTER
        # =======================

        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = lesson_perception.srv.FilterCloudRequest.PASSTHROUGH
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_voxel.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_pass = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful pass through filter operation')

        # PUBLISH PASSTHROUGH FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_passThrough', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_pass.output_cloud)
        print("published: pass through filter response")

#. Save and run from the terminal, repeating steps outlined for the voxel filter.

   Within Rviz, compare PointCloud2 displays based on the ``/kinect/depth_registered/points`` (original camera data) and ``perception_passThrough`` (latest processing step) topics. Part of the original point cloud has been “clipped” out of the latest processing result.


   When you are satisfied with the pass-through filter results, press Ctrl+C to kill the node. There is no need to close or kill the other terminals/nodes.


Plane Segmentation
^^^^^^^^^^^^^^^^^^

This method is one of the most useful for any application where the object is on a flat surface. In order to isolate the objects on a table, you perform a plane fit to the points, which finds the points which comprise the table, and then subtract those points so that you are left with only points corresponding to the object(s) above the table. This is the most complicated PCL method we will be using and it is actually a combination of two: the RANSAC segmentation model, and the extract indices tool. An in depth example can be found on the `PCL Plane Model Segmentation Tutorial <http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation>`__; otherwise you can copy the below code snippet.


#. In ``py_perception_node.cpp``, update the switch statement in ``filterCallback`` to look as shown below:

   .. code-block:: c++

        switch (request.operation)
        {

          case lesson_perception::FilterCloud::Request::VOXELGRID :
          {
            filtered_cloud = voxelGrid(cloud, 0.01);
            break;
          }
          case lesson_perception::FilterCloud::Request::PASSTHROUGH :
          {
            filtered_cloud = passThrough(cloud);
            break;
          }
          case lesson_perception::FilterCloud::Request::PLANESEGMENTATION :
          {
            filtered_cloud = planeSegmentation(cloud);
            break;
          }
          default :
          {
            ROS_ERROR("No valid request found");
            return false;
          }

        }


#. Save and build

   **Edit the Python Code**

#. Copy paste the following code in ``filter_call.py``, after the passthrough filter section.  Keep care to maintain indents:

   .. code-block:: python

        # =======================
        # PLANE SEGMENTATION
        # =======================

        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = lesson_perception.srv.FilterCloudRequest.PLANESEGMENTATION
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_pass.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_seg = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful plane segmentation operation')

        # PUBLISH PLANESEGMENTATION FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_planeSegmentation', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_seg.output_cloud)
        print("published: plane segmentation filter response")


#. Save and run from the terminal, repeating steps outlined for the voxel filter.

   Within Rviz, compare PointCloud2 displays based on the ``/kinect/depth_registered/points`` (original camera data) and ``perception_planeSegmentation`` (latest processing step) topics. Only points lying above the table plane remain in the latest processing result.


   #. When you are done viewing the results you can go back and change the ``setMaxIterations`` and ``setDistanceThreshold`` parameter values to control how tightly the plane-fit classifies data as inliers/outliers, and view the results again. Try using values of ``maxIterations=100`` and ``distThreshold=0.010``

   #. When you are satisfied with the plane segmentation results, use Ctrl+C to kill the node. There is no need to close or kill the other terminals/nodes.


Euclidian Cluster Extraction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This method is useful for any application where there are multiple objects. This is also a complicated PCL method. An in depth example can be found on the `PCL Euclidean Cluster Extration Tutorial <http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction>`__.


#. In ``py_perception_node.cpp``, update the switch statement in ``filterCallback`` to look as shown below:

   .. code-block:: c++

        switch (request.operation)
        {

          case lesson_perception::FilterCloud::Request::VOXELGRID :
          {
            filtered_cloud = voxelGrid(cloud, 0.01);
            break;
          }
          case lesson_perception::FilterCloud::Request::PASSTHROUGH :
          {
            filtered_cloud = passThrough(cloud);
            break;
          }
          case lesson_perception::FilterCloud::Request::PLANESEGMENTATION :
          {
            filtered_cloud = planeSegmentation(cloud);
            break;
          }
          case lesson_perception::FilterCloud::Request::CLUSTEREXTRACTION :
          {
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> temp =clusterExtraction(cloud);
            if (temp.size()>0)
            {
              filtered_cloud = temp[0];
            }
            break;
          }
          default :
          {
            ROS_ERROR("No valid request found");
            return false;
          }

        }


#. Save and build


   **Edit the Python Code**


#. Copy paste the following code in ``filter_call.py`` after the plane segmentation section.  Keep care to maintain indents:

   .. code-block:: python

        # =======================
        # CLUSTER EXTRACTION
        # =======================

        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = lesson_perception.srv.FilterCloudRequest.CLUSTEREXTRACTION
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_seg.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_cluster = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful cluster extraction operation')

        # PUBLISH CLUSTEREXTRACTION FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_clusterExtraction', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_cluster.output_cloud)
        print("published: cluster extraction filter response")


#. Save and run from the terminal, repeating steps outlined for the voxel filter.

   #. When you are satisfied with the cluster extraction results, use Ctrl+C to kill the node. If you are done experimenting with this tutorial, you can kill the nodes running in the other terminals.


Future Study
^^^^^^^^^^^^

The student is encouraged to convert `Exercise 5.1 <http://ros-industrial.github.io/industrial_training/_source/session5/Building-a-Perception-Pipeline.html>`__ into callable functions and further refine the filtering operations.

Furthermore, for simplicity, the python code was repeated for each filtering instance. The student is encouraged to create a loop to handle the publishing instead of repeating large chunks of code.  The student can also leverage the full functionality of the parameter handling instead of just using defaults, can set those from python.  There are several more filtering operations not outlined here, if the student wants practice creating those function calls.
