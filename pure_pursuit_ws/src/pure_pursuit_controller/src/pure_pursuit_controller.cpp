#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include <cmath>

PurePursuitController::PurePursuitController(double wheelbase, double min_ld, double ld_gain, double max_steer)
: wheelbase_(wheelbase), min_ld_(min_ld), ld_gain_(ld_gain), max_steer_(max_steer) {}

void PurePursuitController::setPath(const nav_msgs::Path &path) {
    path_.clear();
    for (auto &pose : path.poses) {
        Point p = {pose.pose.position.x, pose.pose.position.y};
        path_.push_back(p);
    }
}

void PurePursuitController::setPose(const geometry_msgs::PoseStamped &pose) {
    vehicle_pos_.x = pose.pose.position.x;
    vehicle_pos_.y = pose.pose.position.y;

    // Extract heading from quaternion
    double siny = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (pose.pose.orientation.y * pose.pose.orientation.y + pose.pose.orientation.z * pose.pose.orientation.z);
    heading_ = atan2(siny, cosy);
}

int PurePursuitController::findClosestWaypoint() {
    int closest_idx = 0;
    double min_dist = 1e9;
    for (int i = 0; i < path_.size(); i++) {
        double dx = path_[i].x - vehicle_pos_.x;
        double dy = path_[i].y - vehicle_pos_.y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    return closest_idx;
}

double PurePursuitController::computeSteering(double speed) {
    if (path_.empty()) return 0.0;

    double Ld = min_ld_ + ld_gain_ * speed;
    int closest_idx = findClosestWaypoint();

    // Find lookahead point
    Point lookahead = path_.back();
    for (int i = closest_idx; i < path_.size(); i++) {
        double dx = path_[i].x - vehicle_pos_.x;
        double dy = path_[i].y - vehicle_pos_.y;
        if (sqrt(dx*dx + dy*dy) > Ld) {
            lookahead = path_[i];
            break;
        }
    }

    // Transform to vehicle frame
    double dx = lookahead.x - vehicle_pos_.x;
    double dy = lookahead.y - vehicle_pos_.y;
    double local_x =  cos(heading_) * dx + sin(heading_) * dy;
    double local_y = -sin(heading_) * dx + cos(heading_) * dy;

    // Calculate steering angle
    double alpha = atan2(local_y, local_x);
    double delta = atan2(2.0 * wheelbase_ * sin(alpha), Ld);

    // Clamp
    if (delta > max_steer_) delta = max_steer_;
    if (delta < -max_steer_) delta = -max_steer_;
    return delta;
}
    