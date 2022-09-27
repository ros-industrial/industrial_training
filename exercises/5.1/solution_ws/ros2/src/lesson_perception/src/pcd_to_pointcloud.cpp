/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: pcd_to_pointcloud.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**
\author Radu Bogdan Rusu
@b pcd_to_pointcloud is a simple node that loads PCD (Point Cloud Data) files from disk and publishes them as ROS messages on the network.
 **/

// ROS core
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>
#include <sstream>
#include <chrono>

class pcd_to_pointcloud : public rclcpp::Node

{

public:
    pcd_to_pointcloud()
        : Node("pcd_to_pointcloud", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // update potentially remapped topic name for later logging
//        cloud_topic = nh.resolveName(cloud_topic);
        cloud_topic = "cloud_pcd";
        file_name = "";
        interval = 0.0;
        frame_id = "base_link";
        latch = false;
        pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, 1);
        timer = this->create_wall_timer(
            interval,
            std::bind(&pcd_to_pointcloud::timer_callback, this));

        parse_ros_params();
        print_config_info();
        if (!try_load_pointcloud()) {
            RCLCPP_ERROR(this->get_logger(), "Could not load pointcloud");
        }
        print_data_info();
    }

    void publish() {
        RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
        RCLCPP_INFO(this->get_logger(), " * number of points: '%s'", std::to_string(cloud->width * cloud->height));
        RCLCPP_INFO(this->get_logger(), " * frame_id: '%s'", cloud->header.frame_id);
        RCLCPP_INFO(this->get_logger(), " * topic_name: '%s'", cloud_topic);

        cloud->header.stamp = this->get_clock()->now();
        pub->publish(*cloud);
    }

    void timer_callback() {
        // just re-publish
        publish();
    }

    void parse_ros_params() {
        this->get_parameter_or("file_name", file_name_param, rclcpp::Parameter("", file_name));
        this->get_parameter_or("interval", interval_param, rclcpp::Parameter("", interval));
        this->get_parameter_or("frame_id", frame_id_param, rclcpp::Parameter("", frame_id));
        this->get_parameter_or("latch", latch_param, rclcpp::Parameter("", latch));

        file_name = file_name_param.as_string();
        interval = interval_param.as_double();
        frame_id = frame_id_param.as_string();
        latch = latch_param.as_bool();
    }

    void parse_cmdline_args(int argc, char** argv) {
        if (argc > 1) {
            file_name = argv[1];
        }
        if (argc > 2) {
            std::stringstream str(argv[2]);
            double x;
            if (str >> x)
                interval = x;
        }
    }

    bool try_load_pointcloud() {
        if (file_name.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Can't load pointcloud: no file name provided");
            return false;
        }
        else if (pcl::io::loadPCDFile(file_name, *cloud) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse pointcloud from file ('%s')", file_name);
            return false;
        }
        // success: set frame_id appropriately
        cloud->header.frame_id = frame_id;
        return true;
    }

    void print_config_info() {
        RCLCPP_INFO(this->get_logger(), "Recognized the following parameters");
        RCLCPP_INFO(this->get_logger(), " * file_name: '%s'", file_name);
        RCLCPP_INFO(this->get_logger(), " * interval: '%s'", std::to_string(interval));
        RCLCPP_INFO(this->get_logger(), " * frame_id: '%s'", frame_id);
        RCLCPP_INFO(this->get_logger(), " * topic_name: '%s'", cloud_topic);
        RCLCPP_INFO(this->get_logger(), " * latch: '%s'", std::to_string(latch));
    }

    void print_data_info() {
        RCLCPP_INFO(this->get_logger(), "Loaded pointcloud with the following stats");
        RCLCPP_INFO(this->get_logger(), " * number of points: '%s'", std::to_string(cloud->width * cloud->height));
        RCLCPP_INFO(this->get_logger(), " * total size [bytes]: '%s'", std::to_string(cloud->data.size()));
//        RCLCPP_INFO(this->get_logger(), " * channel names: '%s'", std::to_string(pcl::getFieldsList(cloud)));
    }

    rclcpp::Parameter file_name_param, interval_param, frame_id_param, latch_param;

    std::string cloud_topic;
    std::string file_name;
    double interval;
    std::string frame_id;
    bool latch;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    // timer to handle republishing
    rclcpp::WallTimer::SharedPtr timer;
};

int main (int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pcd_to_pointcloud>();
    node->parse_cmdline_args(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
