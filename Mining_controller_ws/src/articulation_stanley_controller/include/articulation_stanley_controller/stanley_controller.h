#ifndef STANLEY_CONTROLLER_H
#define STANLEY_CONTROLLER_H

#include <vector>

class StanleyController {
public:
    // Constructor must match cpp (two parameters)
    StanleyController(double k_gain, double wheelbase);

    // Compute steering
    double computeSteering(double current_x1, double current_y1,
                           double current_theta1, double v,
                           const std::vector<double> &path_x,
                           const std::vector<double> &path_y,
                           const std::vector<double> &path_yaw);

private:
    // Helper: find closest waypoint
    // int closestWaypoint(double x, double y,
    //                     const std::vector<double> &path_x,
    //                     const std::vector<double> &path_y);

    double k_gain_;      // Stanley gain
    double wheelbase_;   // Vehicle wheelbase
};

#endif