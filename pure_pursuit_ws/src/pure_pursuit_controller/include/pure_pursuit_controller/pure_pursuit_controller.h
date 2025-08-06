#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

struct Point {
    double x;
    double y;
};

class PurePursuitController {
public:
    PurePursuitController(double wheelbase, double min_ld, double ld_gain, double max_steer);

    void setPath(const nav_msgs::Path &path);
    void setPose(const geometry_msgs::PoseStamped &pose);
    double computeSteering(double speed);

private:
    std::vector<Point> path_;
    Point vehicle_pos_;
    double heading_;

    double wheelbase_;
    double min_ld_;
    double ld_gain_;
    double max_steer_;

    int findClosestWaypoint();
};

#endif

