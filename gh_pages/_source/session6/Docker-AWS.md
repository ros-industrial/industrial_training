# Docker AWS

## Demo #1 - Run front-end Gazebo host and back-end in Docker
### Setup workspace
#### Front-end (run on host and only contains gui)
in terminal 1 
```
mkdir -p ./training/front_ws/src
cd ./training/front_ws/src
gazebo -v
git clone -b gazebo7 https://github.com/fetchrobotics/fetch_gazebo.git
git clone https://github.com/fetchrobotics/robot_controllers.git
git clone https://github.com/fetchrobotics/fetch_ros.git
cd ..
catkin build fetch_gazebo fetch_description
```
#### Back-end (run in container)
In this step, we will create a docker image that has the executables we need:
 - run /bin/bash in the rosindustrial/core:indigo image then apt-get the package, commiting the result.
 - run /bin/bash in the rosindustrial/core:indigo image then build the package from source, commiting the result.
 - create a docker container using the fetch Dockerfile, which we will perform.
https://gist.github.com/AustinDeric/242c1edf1c934406f59dfd078a0ce7fa
```
cd ../fetch-Dockerfile/
docker build --network=host -t rosindustrial/fetch:indigo .
```
### Running the Demo
#### Run the front-end
Run the front end in terminal 1: 
```
source devel/setup.bash
roslaunch fetch_gazebo playground.launch
```
#### Run the backend
There are multiple ways to perform this:
 - run /bin/bash in the fetch container and manually run the demo node.
 - run the demo node directly in the container, which is the method we will perform
 
Run the back end in terminal 2:
```
docker run --network=host rosindustrial/fetch:indigo roslaunch fetch_gazebo_demo demo.launch
```

## Demo #2 - Run front-end on a web-server and back-end in docker
start the environment
```
docker run --network=host rosindustrial/fetch:indigo roslaunch fetch_gazebo playground.launch headless:=true gui:=false
```
run the gazebo web server:
```
docker run -v "/home/ros-industrial/roscloud/training/front_ws/src/fetch_gazebo/fetch_gazebo/models/test_zone/meshes/:/root/gzweb/http/client/assets/test_zone/meshes/" -v "/home/ros-industrial/roscloud/training/front_ws/src/fetch_ros/fetch_description/meshes:/root/gzweb/http/client/assets/fetch_description/meshes" -it --network=host giodegas/gzweb /bin/bash
```
then run the server:
```
/root/gzweb/start_gzweb.sh && gzserver
```

run the demo in terminal 3:
```
docker run --network=host fetch roslaunch fetch_gazebo_demo demo.launch
```
## Demo #3 Robot Web Tools
In this demo we will run an industrial robot URDF viewable in a browser
In terminal 1 we will load a robot to the parameter server
```
mkdir -p abb_ws/src
git clone -b kinetic-devel https://github.com/ros-industrial/abb.git
docker run -v "/home/ros-industrial/roscloud/training/abb_ws:/abb_ws" --network=host -it rosindustrial/core:melodic /bin/bash
cd abb_ws
catkin build
source devel/setup.bash
roslaunch abb_irb5400_support load_irb5400.launch
```

in terminal 2 we will start the robot web tools:
```
docker run --network=host rosindustrial/viz:kinetic roslaunch viz.launch
```
in terminal 3 we will launch the webserver
first we need to start a www folder
```
cp -r abb_ws/src/abb/abb_irb5400_support/ www/
```
<script src="https://gist.github.com/AustinDeric/e806e6676e6c80590b8562633c7220a4.js"></script>
<script src="https://gist.github.com/AustinDeric/ed046693bed9e55dfbe546fe8d479284.js"></script>

```
docker run -v "/home/ros-industrial/roscloud/training/www:/data/www" -v "/home/ros-industrial/roscloud/training/nginx_conf/:/etc/nginx/local/" -it --network=host rosindustrial/nginx:latest /bin/bash
nginx -c /etc/nginx/local/nginx.conf
```
