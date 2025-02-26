#ifndef TRAJECTORY_PUB_SAVER_HPP
#define TRAJECTORY_PUB_SAVER_HPP

// Include necessary headers for ROS2 and message types
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "trajectory_pkg/srv/save_trajectory.hpp"
#include "json/json.h"
#include <fstream>
#include <vector>
#include <cmath>

// Define the TrajectoryPubSaver class that records and visualizes trajectory data
class TrajectoryPubSaver : public rclcpp::Node {

public:
    // Constructor 
    TrajectoryPubSaver();

private: 
    // Store recorded trajectory data
    std::vector<nav_msgs::msg::Odometry::SharedPtr> path_data;
    nav_msgs::msg::Odometry::SharedPtr latest_pose;
    
    // ROS2 interfaces: subscribers, timers, publishers, and services
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_listener;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visual_publisher;
    rclcpp::Service<trajectory_pkg::srv::SaveTrajectory>::SharedPtr storage_service;

    // Parameters for topic names and frame of reference
    std::string pose_source, visualization_topic, reference_frame;
    double refresh_rate;

    // Callback function to handle incoming odometry messages
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Function to periodically record trajectory data
    void record_path();
    
    // Function to visualize the recorded trajectory
    void render_path();
    
    // Service function to save trajectory data to a file
    void store_path(const std::shared_ptr<trajectory_pkg::srv::SaveTrajectory::Request> request,
                    std::shared_ptr<trajectory_pkg::srv::SaveTrajectory::Response> response);
};

#endif // TRAJECTORY_PUB_SAVER_HPP
