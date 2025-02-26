#ifndef PATH_LOADER_HPP
#define PATH_LOADER_HPP

// Required Headers
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "trajectory_pkg/srv/read_trajectory.hpp"
#include <json/json.h>
#include <fstream>
#include <cmath>

// TrajectoryReader class to manage saved trajectory visualization
class TrajectoryReader : public rclcpp::Node {
    public:
        // Constructor
        TrajectoryReader();

    private:

        // Variables to store frame and topic name
        std::string reference_frame;
        std::string visualization_topic;

        // Service callback to read a trajectory file and publish markers
        void handle_trajectory_request(
            const std::shared_ptr<trajectory_pkg::srv::ReadTrajectory::Request> request,
            std::shared_ptr<trajectory_pkg::srv::ReadTrajectory::Response> response);
        
        // Function to visualize the trajectory as markers
        void display_path(const Json::Value &trajectory_data);
        
        // Publisher for trajectory markers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_publisher;
        
        // Service to load a saved trajectory
        rclcpp::Service<trajectory_pkg::srv::ReadTrajectory>::SharedPtr trajectory_service;
        
};

#endif // PATH_LOADER_HPP