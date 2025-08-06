
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <ros/package.h>
#include <articulation_stanley_controller/stanley_controller.h>

void transform2D(double x_local, double y_local,
    double x_ref, double y_ref, double theta,
    double& x_out, double& y_out)
{
x_out = x_ref + std::cos(theta) * x_local - std::sin(theta) * y_local;
y_out = y_ref + std::sin(theta) * x_local + std::cos(theta) * y_local;
}

// Load waypoints from CSV file
    std::vector<geometry_msgs::Point> loadCSVPath(const std::string& file_path) {
    std::vector<geometry_msgs::Point> waypoints;
    std::ifstream file(file_path);

    if (!file.is_open()) {
        ROS_ERROR("Could not open file: %s", file_path.c_str());
        return waypoints;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');

        try {
            geometry_msgs::Point pt;
            pt.x = std::stod(x_str);
            pt.y = std::stod(y_str);
            waypoints.push_back(pt);
        } catch (...) {
            ROS_WARN("Skipping invalid line: %s", line.c_str());
        }
    }

    file.close();
    return waypoints;
    }


int main(int argc, char** argv) {
    ros::init(argc, argv, "Articulated_vehicle_R2900G_model");
    ros::NodeHandle nh;

    ros::Publisher front_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("front_unit_pose", 1);
    ros::Publisher rear_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("rear_unit_pose", 1);


    ros::Publisher front_path_pub = nh.advertise<nav_msgs::Path>("front_unit_path", 1);
    ros::Publisher rear_path_pub = nh.advertise<nav_msgs::Path>("rear_unit_path", 1);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("vehicle_body_marker", 10);
    
    ros::Publisher ref_path_pub = nh.advertise<nav_msgs::Path>("reference_path", 1);

    nav_msgs::Path front_path, rear_path;
    front_path.header.frame_id = "map";
    rear_path.header.frame_id = "map";

    // Load critical path from CSV 
    std::string csv_path = ros::package::getPath("articulation_stanley_controller") + "/data/infinity_path.csv";
    //std::string csv_path = ros::package::getPath("articulation_stanley_controller") + "/data/square_path_0_1_gap.csv";
    //std::string csv_path = ros::package::getPath("articulation_stanley_controller") + "/data/oval_path.csv";
    std::vector<geometry_msgs::Point> waypoints = loadCSVPath(csv_path);

    nav_msgs::Path ref_path;
    ref_path.header.frame_id = "map";
    for (const auto& pt : waypoints) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position = pt;
        pose.pose.orientation.w = 0.5;
        ref_path.poses.push_back(pose);
    }

    std::vector<double> path_x, path_y, path_yaw;
for (size_t i = 0; i < waypoints.size(); ++i) {
    path_x.push_back(waypoints[i].x);
    path_y.push_back(waypoints[i].y);

    if (i < waypoints.size() - 1) {
        double dx = waypoints[i+1].x - waypoints[i].x;
        double dy = waypoints[i+1].y - waypoints[i].y;
        path_yaw.push_back(std::atan2(dy, dx));
    } else {
        path_yaw.push_back(path_yaw.back()); // last yaw same as previous
    }
}


    // Define shared color
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.5;
    color.b = 0.0;
    color.a = 1.0;

    // === Define points for both pentagons outside the loop ===
    std::vector<geometry_msgs::Point> front_polygon;
    std::vector<geometry_msgs::Point> rear_polygon;
    geometry_msgs::Point p;

    // Front pentagon (pointing right)
    p.x =  1.8; p.y =  1.5; front_polygon.push_back(p);
    p.x =  1.8; p.y = -1.5; front_polygon.push_back(p);
    p.x =  0.5; p.y = -1.5; front_polygon.push_back(p);
    p.x =  0.0; p.y =  0.0; front_polygon.push_back(p);  // Nose tip
    p.x =  0.5; p.y =  1.5; front_polygon.push_back(p);
    p.x =  1.8; p.y =  1.5; front_polygon.push_back(p);  // Close loop

    // // Rear pentagon (pointing left)
    p.x = -1.8; p.y =  1.5; rear_polygon.push_back(p);
    p.x = -1.8; p.y = -1.5; rear_polygon.push_back(p);
    p.x = -0.5; p.y = -1.5; rear_polygon.push_back(p);
    p.x =  0.0; p.y =  0.0; rear_polygon.push_back(p);   // Shared tip
    p.x = -0.5; p.y =  1.5; rear_polygon.push_back(p);
    p.x = -1.8; p.y =  1.5; rear_polygon.push_back(p);   // Close loop

    
    
     // Pose history arrays
    std::vector<double> x1_arr, y1_arr, theta1_arr;
    std::vector<double> x2_arr, y2_arr, theta2_arr;

    // // Initial conditions
    // double x1 = 0.0, y1 = 0.0, theta1 = 0.0;
    // double x2 = 0.0, y2 = 0.0, theta2 = 0.0;
    double phi = 40 *  3.14 / 180;
    double phi_dot = 0.0;

    
    
    double l1 = 1.89, l2 = 1.89, l=3.78, v = 1.0;
    // double dt = 0.1;
    // double time = 0.0;
    
    

    // Initial positions
    double x1 = 0.0, y1 = 0.0, theta1 = 0.0; 
    double x2 = 0.0, y2 = 0.0, theta2 = 0.0; 
   
    //double v = 1.0;     
    double dt = 0.1;

    // StanleyController controller(1.0, 30.0);
    StanleyController controller(2.0, l1);
    ros::Rate rate(20);
    while (ros::ok()) {

        double steer = controller.computeSteering(x1, y1, theta1, v, path_x, path_y, path_yaw);
        phi = steer;


        // === FRONT VEHICLE KINEMATICS ===
        double theta1_dot = (v * std::sin(phi) + l2 * phi_dot) / (l1 * std::cos(phi) + l2);
        theta1 += theta1_dot * dt;
        x1 += v * std::cos(theta1) * dt;
        y1 += v * std::sin(theta1) * dt;

        // === Analyse the output of front body ===
        
        std::cout << "x1: " << x1 << "\n";
        std::cout << "y1: " << y1 << "\n";
        std::cout << "Theta Dot: " << theta1 << "\n";

        // === REAR VEHICLE KINEMATICS ===       
        double theta2_dot = (v / l2) * sin(theta1 - theta2);
        theta2 += theta2_dot * dt;
        x2 = x1 - 0.01 * std::cos(theta1) - 0.01 * std::cos(theta2);
        y2 = y1 - 0.01 * std::sin(theta1) - 0.01 * std::sin(theta2); 
        
        // === Analyse the output of front body ===

        std::cout << "x2: " << x2 << "\n";
        std::cout << "y2: " << y2 << "\n";
        
        // Store pose history in arrays
        x1_arr.push_back(x1);
        y1_arr.push_back(y1);
        theta1_arr.push_back(theta1);

        x2_arr.push_back(x2);
        y2_arr.push_back(y2);
        theta2_arr.push_back(theta2);

        // Front pose
        geometry_msgs::PoseStamped front_pose;
        front_pose.header.frame_id = "map";
        front_pose.header.stamp = ros::Time::now();
        front_pose.pose.position.x = x1;
        front_pose.pose.position.y = y1;
        front_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta1);
        front_path.poses.push_back(front_pose);

        // Rear pose
        geometry_msgs::PoseStamped rear_pose;
        rear_pose.header.frame_id = "map";
        rear_pose.header.stamp = ros::Time::now();
        rear_pose.pose.position.x = x2;
        rear_pose.pose.position.y = y2;
        rear_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta2);
        rear_path.poses.push_back(rear_pose);

        // === Front Vehicle Box Marker ===
        visualization_msgs::Marker marker_front;
        marker_front.header.frame_id = "map";
        marker_front.header.stamp = ros::Time::now();
        marker_front.ns = "vehicle_body_marker";
        marker_front.id = 0;
        marker_front.type = visualization_msgs::Marker::LINE_STRIP;
        marker_front.action = visualization_msgs::Marker::ADD;
        marker_front.scale.x = 0.05;

        marker_front.color.r = 1.0;
        marker_front.color.g = 0.5;
        marker_front.color.b = 0.0;
        marker_front.color.a = 1.0;
        
        marker_front.points = front_polygon;


        // Rear pentagon marker
        visualization_msgs::Marker marker_rear = marker_front;
        marker_rear.id = 1;

        marker_rear.color.r = 0.0;
        marker_rear.color.g = 0.3;
        marker_rear.color.b = 1.0;
        marker_rear.color.a = 1.0;
        
        marker_rear.points = rear_polygon;


        marker_front.points.clear();

    for (const auto& pt : front_polygon) {
        double xg, yg;
        transform2D(pt.x, pt.y, x1, y1, theta1, xg, yg);
        geometry_msgs::Point p;
        p.x = xg;
        p.y = yg;
        p.z = 0.0;
    marker_front.points.push_back(p);
}

       marker_rear.points.clear();

    for (const auto& pt : rear_polygon) {
        double xg, yg;
        transform2D(pt.x, pt.y, x2, y2, theta2, xg, yg);
        geometry_msgs::Point p;
        p.x = xg;
        p.y = yg;
        p.z = 0.0;
    marker_rear.points.push_back(p);
}

        // Publish paths
        front_path_pub.publish(front_path);
        rear_path_pub.publish(rear_path);
        ref_path_pub.publish(ref_path);

        // Publish Pose
        front_pose_pub.publish(front_pose);
        rear_pose_pub.publish(rear_pose);

        // Publish Markers
        marker_pub.publish(marker_front);
        marker_pub.publish(marker_rear);

        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
