#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <string>
#include <vector>
#include <my_cpp_package/SaveTrajectory.h>

class TrajectoryPublisherSaverNode {
public:
    TrajectoryPublisherSaverNode() {
        file_path_ = "trajectory_data.csv"; // Relative file path
        // Subscribe to odometry topic to get robot's pose
        odom_sub_ = nh_.subscribe("/odom", 10, &TrajectoryPublisherSaverNode::odomCallback, this);
        // Advertise marker array topic for visualization
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_marker_array", 10);

       // Advertise the service for users to specify the file name and duration
        save_trajectory_service_ = nh_.advertiseService("save_trajectory", &TrajectoryPublisherSaverNode::saveTrajectoryCallback, this); 
}  


bool saveTrajectoryCallback(my_cpp_package::SaveTrajectory::Request& req,
                             my_cpp_package::SaveTrajectory::Response& res) {
    // Extract the file name and duration from the service request
    std::string file_name = req.file_name;
    int duration_seconds = req.duration_seconds;

    // Print the extracted values for verification
    ROS_INFO("File Name: %s", file_name.c_str());
    ROS_INFO("Duration (seconds): %d", duration_seconds);

    // Set the response to indicate success
    res.success = true;
    write_file_flag = 1;

    // Open the trajectory data file for reading
    std::ifstream trajectory_data_file(file_path_);
    if (!trajectory_data_file.is_open()) {
        ROS_ERROR("Failed to open trajectory data file: %s", file_path_.c_str());
        return false;
    }

    // Skip the header line
    std::string header;
    std::getline(trajectory_data_file, header);

    // Read the last line of the trajectory data file to get the last timestamp
    int last_timestamp = 0; // Initialize last timestamp to 0
    std::string line;
    while (std::getline(trajectory_data_file, line)) {
        std::istringstream iss(line);
        std::string token;
        int timestamp = 0; // Initialize timestamp for each line
        int token_count = 0; // Keep track of the token count
        // Split the line by commas and get the last token (timestamp)
        while (std::getline(iss, token, ',')) {
            if (!token.empty()) {
                try {
                    if (token_count == 7) { // Timestamp is the 8th token
                        timestamp = std::stoi(token); // Update timestamp
                    }
                } catch (const std::invalid_argument& e) {
                    ROS_ERROR("Invalid timestamp format: %s", token.c_str());
                    return false; // Return false to indicate failure
                } catch (const std::out_of_range& e) {
                    ROS_ERROR("Timestamp out of range: %s", token.c_str());
                    return false; // Return false to indicate failure
                }
                token_count++;
            }
        }
        // Update the last timestamp
        last_timestamp = timestamp;
    }

    // Calculate the new timestamp by subtracting duration_seconds
    int new_timestamp = last_timestamp - duration_seconds;
    
    // Close the trajectory data file
    trajectory_data_file.close();

    // Open the trajectory data file again for reading
    trajectory_data_file.open(file_path_);
    if (!trajectory_data_file.is_open()) {
        ROS_ERROR("Failed to open trajectory data file: %s", file_path_.c_str());
        return false;
    }

    // Create a new file with the provided file name
    std::ofstream new_file(file_name);
    if (!new_file.is_open()) {
        ROS_ERROR("Failed to create file: %s", file_name.c_str());
        trajectory_data_file.close();
        return false;
    } else {
        // Write the header to the new file
        const std::string header = "Position X,Position Y,Position Z,Orientation X,Orientation Y,Orientation Z,Orientation W,Timestamp\n";
        new_file << header;
        ROS_INFO("New file created: %s", file_name.c_str());
    }

    // Skip the header line in the new file
    std::getline(trajectory_data_file, header);

    // Read the lines from the trajectory data file and copy the data to the new file
    while (std::getline(trajectory_data_file, line)) {
        std::istringstream iss(line);
        std::string token;
        int timestamp = 0; // Initialize timestamp for each line
        int token_count = 0; // Keep track of the token count
        // Split the line by commas and get the last token (timestamp)
        while (std::getline(iss, token, ',')) {
            if (!token.empty()) {
                try {
                    if (token_count == 7) { // Timestamp is the 8th token
                        timestamp = std::stoi(token); // Update timestamp
                    }
                } catch (const std::invalid_argument& e) {
                    ROS_ERROR("Invalid timestamp format: %s", token.c_str());
                    ROS_ERROR("Full line: %s", line.c_str()); // Print the full line for debugging
                    return false; // Return false to indicate failure
                } catch (const std::out_of_range& e) {
                    ROS_ERROR("Timestamp out of range: %s", token.c_str());
                    ROS_ERROR("Full line: %s", line.c_str()); // Print the full line for debugging
                    return false; // Return false to indicate failure
                }
                token_count++;
            }
        }
        // Compare the extracted timestamp with the new timestamp
        if (timestamp >= new_timestamp) {
            // Copy the line to the new file
            new_file << line << std::endl;
            // Print a message indicating that the line has been copied
            ROS_INFO("Copied line: %s", line.c_str());
        }
    }

    // Close the trajectory data file
    trajectory_data_file.close();

    // Close the new file
    new_file.close();

    return true; // Return true to indicate success
}





  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    static geometry_msgs::Pose last_pose;  // Variable to store the last received pose

    // Check if the current pose is different from the previous one
    
        // Open the trajectory data file for appending
        std::ofstream outfile(file_path_, std::ios_base::app);
        if (!outfile.is_open()) {
            ROS_ERROR("Failed to open file for writing.");
            return;
        }

        // Write headers if the file is empty
        if (outfile.tellp() == 0) {
            outfile << "Position X,Position Y,Position Z,Orientation X,Orientation Y,Orientation Z,Orientation W,Timestamp" << std::endl;
        }




       if(write_file_flag == 0)
       {
        // Write the trajectory data to the file in CSV format
        outfile << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "," << msg->pose.pose.position.z << ",";
        outfile << msg->pose.pose.orientation.x << "," << msg->pose.pose.orientation.y << "," << msg->pose.pose.orientation.z << "," << msg->pose.pose.orientation.w << ",";
        
        // Extract and convert the timestamp to seconds
        ros::Time timestamp = msg->header.stamp;
        int64_t secs = timestamp.sec;

        // Write the timestamp data to the file
        outfile << secs << std::endl;
       }
  
        // Publish markers for visualization
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; // Set the frame ID
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
        point.x = msg->pose.pose.position.x;
        point.y = msg->pose.pose.position.y;
        point.z = msg->pose.pose.position.z;
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
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher marker_pub_;
    ros::ServiceServer save_trajectory_service_;
    std::string file_path_;
    float duration;
    int marker_id_ = 0; // Counter for marker IDs
    int write_file_flag = 0;
    std::vector<geometry_msgs::Point> trajectory_points_; // Vector to store trajectory points
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher_saver_node");

    // Create an instance of TrajectoryPublisherSaverNode
    TrajectoryPublisherSaverNode publisher_saver_node;
    ros::spin();
    return 0;
}