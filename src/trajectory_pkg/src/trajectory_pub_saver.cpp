#include "trajectory_pkg/trajectory_pub_saver.hpp"

// Constructor: Initializes the node and sets up parameters, subscriptions, publishers, and services.
TrajectoryPubSaver::TrajectoryPubSaver() : Node("trajectory_pub_saver") {
    
    // Declare ROS2 parameters with default values
    this->declare_parameter<std::string>("visualization_topic", "/path_visual");
    this->declare_parameter<std::string>("reference_frame", "odom");
    this->declare_parameter<std::string>("pose_topic", "/odom");
    this->declare_parameter<double>("update_rate", 8.0);

    // Retrieve parameter values
    visualization_topic = this->get_parameter("visualization_topic").as_string();
    pose_source = this->get_parameter("pose_topic").as_string();
    reference_frame = this->get_parameter("reference_frame").as_string();
    refresh_rate = this->get_parameter("update_rate").as_double();

    // Subscribe to odometry data for tracking trajectory
    pose_listener = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_source, 10, std::bind(&TrajectoryPubSaver::pose_callback, this, std::placeholders::_1));
    
    // Set up a timer to periodically record trajectory data
    timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / refresh_rate)),
        std::bind(&TrajectoryPubSaver::record_path, this));

    // Publisher for visualizing saved trajectory as markers
    visual_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic, 10);

    // Service to save the recorded trajectory to a file
    storage_service = this->create_service<trajectory_pkg::srv::SaveTrajectory>(
        "store_path", std::bind(&TrajectoryPubSaver::store_path, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Trajectory publisher saver node is active.");
    RCLCPP_INFO(this->get_logger(), "Use /store_path service to save the recorded path within a specified time duration.");
}

// Callback function: Updates the latest received pose from odometry messages
void TrajectoryPubSaver::pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_pose = msg;
}

// Function: Stores the latest pose in the trajectory if available
void TrajectoryPubSaver::record_path() {
    if (latest_pose) {
        path_data.push_back(latest_pose);
        render_path(); // Update visualization
    }
}

// Function: Publishes the recorded trajectory as markers for visualization
void TrajectoryPubSaver::render_path() {
    visualization_msgs::msg::MarkerArray path_markers;

    for (size_t i = 0; i < path_data.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = reference_frame;
        marker.header.stamp = this->now();
        marker.ns = "path_visualization";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = path_data[i]->pose.pose;
        
        // Set marker size
        marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
        
        // Set marker color (green)
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        
        // Set lifetime for markers
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
        
        path_markers.markers.push_back(marker);
    }
    visual_publisher->publish(path_markers);
}

// Function: Saves the recorded trajectory to a JSON file upon service request
void TrajectoryPubSaver::store_path(const std::shared_ptr<trajectory_pkg::srv::SaveTrajectory::Request> request,
                              std::shared_ptr<trajectory_pkg::srv::SaveTrajectory::Response> response) {
    try {
        if (path_data.empty()) {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "No path data available for saving.");
            return;
        }

        auto latest_odom_msg = path_data.back();
        auto current_time_ns = latest_odom_msg->header.stamp.sec * 1e9 + latest_odom_msg->header.stamp.nanosec;

        // Filter trajectory data within the requested duration
        std::vector<nav_msgs::msg::Odometry::SharedPtr> filtered_path;
        for (const auto &p : path_data) {
            auto timestamp_ns = p->header.stamp.sec * 1e9 + p->header.stamp.nanosec;

            // Ensure the timestamp is within the valid range
            if (timestamp_ns <= current_time_ns && (current_time_ns - timestamp_ns) <= request->duration * 1e9) {
                filtered_path.push_back(p);
            }
        }

        // Create a JSON object to store path data
        Json::Value path_json;
        for (const auto &entry : filtered_path) {
            Json::Value node;
            node["x"] = entry->pose.pose.position.x;
            node["y"] = entry->pose.pose.position.y;
            node["yaw"] = std::atan2(2.0 * (entry->pose.pose.orientation.w * entry->pose.pose.orientation.z),
                                      1.0 - 2.0 * (entry->pose.pose.orientation.z * entry->pose.pose.orientation.z));
            path_json.append(node);
        }

        // Write JSON data to a file
        std::ofstream file(request->filename + ".json");
        file << path_json;
        
        response->success = !filtered_path.empty();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Path saved to %s.json", request->filename.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "No path data saved.");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save path: %s", e.what());
        response->success = false;
    }
}
       

// Main function: Initializes ROS2, starts the node, and keeps it running
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPubSaver>());
    rclcpp::shutdown();
    return 0;
}