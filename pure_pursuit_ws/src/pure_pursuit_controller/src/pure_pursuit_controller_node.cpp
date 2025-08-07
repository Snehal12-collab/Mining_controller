
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <pure_pursuit_controller/pure_pursuit_controller.h> // New header

// Transform from local car frame to global map frame
void transform2D(double x_local, double y_local, double x_ref, double y_ref, double theta, double &x_out, double &y_out)
{
    x_out = x_ref + std::cos(theta) * x_local - std::sin(theta) * y_local;
    y_out = y_ref + std::sin(theta) * x_local + std::cos(theta) * y_local;
}

// Load waypoints from CSV
std::vector<geometry_msgs::Point> loadCSVPath(const std::string &file_path)
{
    std::vector<geometry_msgs::Point> waypoints;
    std::ifstream file(file_path);

    if (!file.is_open())
    {
        ROS_ERROR("Could not open file: %s", file_path.c_str());
        return waypoints;
    }

    std::string line;
    bool first = true;
    while (std::getline(file, line))
    {
        if (first) { first = false; continue; } // Skip header
        std::stringstream ss(line);
        std::string x_str, y_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');

        try
        {
            geometry_msgs::Point pt;
            pt.x = std::stod(x_str);
            pt.y = std::stod(y_str);
            waypoints.push_back(pt);
        }
        catch (...)
        {
            ROS_WARN("Skipping invalid line: %s", line.c_str());
        }
    }

    file.close();
    return waypoints;
}

// ================= MAIN =================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_controller_node");
    ros::NodeHandle nh;

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("car_pose", 1);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("car_path", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("car_marker", 1);
    ros::Publisher ref_path_pub = nh.advertise<nav_msgs::Path>("reference_path", 1);

    // Load CSV waypoints
    std::string csv_path = ros::package::getPath("pure_pursuit_controller") + "/data/infinity_path.csv";
    // std::string csv_path = ros::package::getPath("pure_pursuit_controller") + "/data/square_path_0_1_gap.csv";
    std::vector<geometry_msgs::Point> waypoints = loadCSVPath(csv_path);

    // Build reference path (nav_msgs::Path)
    nav_msgs::Path ref_path;
    ref_path.header.frame_id = "map";
    for (const auto &pt : waypoints)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position = pt;
        pose.pose.orientation.w = 1.0;
        ref_path.poses.push_back(pose);
    }

    // Initialize Pure Pursuit Controller (wheelbase=2.5, min_ld=1.0, ld_gain=0.5, max_steer=0.6)
    PurePursuitController controller(1.5, 0.5, 0.5, 0.6);
    controller.setPath(ref_path);

    // Car polygon shape (local coordinates)
    std::vector<geometry_msgs::Point> car_shape;
    geometry_msgs::Point pt;
    pt.x = 1.0; pt.y = 0.5; car_shape.push_back(pt);
    pt.x = 1.0; pt.y = -0.5; car_shape.push_back(pt);
    pt.x = -1.0; pt.y = -0.5; car_shape.push_back(pt);
    pt.x = -1.0; pt.y = 0.5; car_shape.push_back(pt);
    pt.x = 1.0; pt.y = 0.5; car_shape.push_back(pt);

    // Vehicle state (initial conditions)
    double x = 0.0, y = 0.0, yaw = 0.0;
    double v = 1.0;       // constant speed
    double dt = 0.1;

    // Path to be published for car
    nav_msgs::Path car_path;
    car_path.header.frame_id = "map";

    ros::Rate rate(20);
    while (ros::ok())
    {
        // Update vehicle pose in controller
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        controller.setPose(pose_msg);

        // Compute steering angle
        double delta = controller.computeSteering(v);

        // Update vehicle state (bicycle model)
        x += v * std::cos(yaw) * dt;
        y += v * std::sin(yaw) * dt;
        yaw += (v / 2.5) * std::tan(delta) * dt;

        // Publish vehicle pose
        pose_pub.publish(pose_msg);

        // Append to car path
        car_path.poses.push_back(pose_msg);
        path_pub.publish(car_path);

        // Publish car shape as marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "car_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 0.9;
        marker.color.b = 0.2;
        marker.color.a = 1.0;

        marker.points.clear();
        for (const auto &p_local : car_shape)
        {
            double xg, yg;
            transform2D(p_local.x, p_local.y, x, y, yaw, xg, yg);
            geometry_msgs::Point p_global;
            p_global.x = xg;
            p_global.y = yg;
            p_global.z = 0.0;
            marker.points.push_back(p_global);
        }
        marker_pub.publish(marker);

        // Publish reference path
        ref_path_pub.publish(ref_path);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
