#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <string>
#include <vector>
#include <my_cpp_package/SaveTrajectory.h>


class TrajectoryReaderPublisherNode {
public:
    TrajectoryReaderPublisherNode(const std::string& filename): filename_(filename) {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle();

        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("saved_trajectory_marker_array", 10);

        // Read trajectory data from file
        readTrajectoryFromFile();
    }

    // Function to read trajectory data from the file
    void readTrajectoryFromFile() {
    // Reset marker ID
    //marker_id_ = 0;

    // Open the file
    std::ifstream file(filename_);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open trajectory file: %s", filename_.c_str());
        return;
    }

    // Skip the first line (header)
    std::string header;
    std::getline(file, header);

    // Read the file line by line
    std::string line;
    while (std::getline(file, line)) {
        // Parse the line and extract trajectory data
        std::istringstream iss(line);
        std::string token;
        std::vector<double> values;
        while (std::getline(iss, token, ',')) {
            // Convert token to double and add to values vector
            values.push_back(std::stod(token));
        }


        // Create marker array
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom"; // Set the frame ID
        marker.header.stamp = ros::Time::now(); // Set the timestamp
        marker.ns = "trajectory"; // Set the namespace
        marker.id = marker_id_; // Set the marker ID
        marker.type = visualization_msgs::Marker::LINE_STRIP; // Set the marker type to line strip
        marker.action = visualization_msgs::Marker::ADD; // Set the action to add
        marker.pose.orientation.w = 1.0; // Set the orientation
        marker.scale.x = 0.05; // Set the line width
        marker.color.r = 1.0; // Set the color to red
        marker.color.a = 1.0; // Set the alpha (transparency)
        marker.lifetime = ros::Duration(); // Marker lasts forever

        // Add points to the marker representing the trajectory
        geometry_msgs::Point point;
        point.x = values[0]; // Use trajectory data from the file
        point.y = values[1];
        point.z = values[2];
        trajectory_points_.push_back(point); // Store the current point
        // Add all points stored in the trajectory
        for (const auto& previous_point : trajectory_points_) {
            marker.points.push_back(previous_point);
             
        }

        // Push the marker to the array
        marker_array.markers.push_back(marker);
        // Publish the marker array
        marker_pub_.publish(marker_array);
        // Increment marker ID for the next marker
        marker_id_++;         

        // Sleep to control the publish rate
        ros::Duration(0.1).sleep(); // Sleep for 0.1 seconds

    }

    // Close the file
    file.close();
}


private:
    ros::NodeHandle nh_;
    std::string filename_;
    ros::Publisher marker_pub_;
    int marker_id_ = 0;
    std::vector<geometry_msgs::Point> trajectory_points_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_reader_publisher_node");

    // Check if a filename argument is provided
    std::string filename;
    if (argc > 1) {
        filename = argv[1];
    } else {
        ROS_WARN("No filename provided. Using default filename.");
        filename = "my_trajectory.csv";
    }

    // Create an instance of TrajectoryReaderPublisherNode with the filename argument
    TrajectoryReaderPublisherNode reader_publisher_node(filename);

    ros::spin();
    return 0;
}