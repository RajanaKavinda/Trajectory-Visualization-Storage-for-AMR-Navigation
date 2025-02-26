#include "trajectory_pkg/trajectory_reader.hpp"

// Constructor initializing the node
TrajectoryReader::TrajectoryReader() : Node("trajectory_reader") {
    // Declare parameters with default values
    declare_parameter("visualization_topic", "/loaded_path_markers");
    declare_parameter("reference_frame", "odom");

    // Retrieve parameter values
    reference_frame = get_parameter("reference_frame").as_string();
    visualization_topic = get_parameter("visualization_topic").as_string();

    // Setup publisher for markers
    trajectory_publisher = create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic, 10);

    // Register service for loading trajectories
    trajectory_service = create_service<trajectory_pkg::srv::ReadTrajectory>(
        "read_path", std::bind(&TrajectoryReader::handle_trajectory_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Trajectory reader node is initialized.");
    RCLCPP_INFO(get_logger(), "Use /read_path service to visualize saved paths.");
}

void TrajectoryReader::handle_trajectory_request(
    const std::shared_ptr<trajectory_pkg::srv::ReadTrajectory::Request> request,
    std::shared_ptr<trajectory_pkg::srv::ReadTrajectory::Response> response) {
    try {
        // Read trajectory data from file
        std::ifstream file(request->filename + ".json");
        Json::Value trajectory_data;
        file >> trajectory_data;

        if (trajectory_data.empty()) {
            RCLCPP_WARN(get_logger(), "Loaded file is empty.");
            response->success = false;
            response->message = "No data available in file.";
            return;
        }

        // Publish markers for visualization
        display_path(trajectory_data);
        RCLCPP_INFO(get_logger(), "Trajectory loaded from %s and published.", request->filename.c_str());
        response->success = true;
        response->message = "Trajectory successfully visualized.";
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Failed to process trajectory: %s", e.what());
        response->success = false;
        response->message = "Error loading trajectory.";
    }
}

void TrajectoryReader::display_path(const Json::Value &trajectory_data) {
    visualization_msgs::msg::MarkerArray marker_array;

    for (Json::ArrayIndex i = 0; i < trajectory_data.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = reference_frame;
        marker.header.stamp = now();
        marker.ns = "visualized_path";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = trajectory_data[i]["x"].asFloat();
        marker.pose.position.y = trajectory_data[i]["y"].asFloat();

        float yaw = trajectory_data[i]["yaw"].asFloat();
        marker.pose.orientation.z = std::sin(yaw / 2);
        marker.pose.orientation.w = std::cos(yaw / 2);

        marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_array.markers.push_back(marker);
    }

    trajectory_publisher->publish(marker_array);
    RCLCPP_INFO(get_logger(), "Published trajectory visualization.");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}