#include "articulation_stanley_controller/stanley_controller.h"
#include <cmath>
#include <limits>

StanleyController::StanleyController(double k_gain, double wheelbase)
    : k_gain_(k_gain), wheelbase_(wheelbase) {}

double StanleyController::computeSteering(double x1, double y1, double theta1, double v,
                                          const std::vector<double>& path_x,
                                          const std::vector<double>& path_y,
                                          const std::vector<double>& path_yaw) {
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path_x.size(); ++i) {
        double dx = path_x[i] - x1;
        double dy = path_y[i] - y1;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    double dx = path_x[closest_idx] - x1;
    double dy = path_y[closest_idx] - y1;
    double heading_error = path_yaw[closest_idx] - theta1;
    double cross_track_error = dy * std::cos(path_yaw[closest_idx]) - dx * std::sin(path_yaw[closest_idx]);

    double steer = heading_error + std::atan2(k_gain_ * cross_track_error, v + 1e-3);
    return steer;
}

