#ifndef PLANNING_PLANNER_BEZIER_PATH_H_
#define PLANNING_PLANNER_BEZIER_PATH_H_

#include "planning/planner/path.h"

/**
 * @brief The BezierPath class
 */
class BezierPath : public Path {
public:

    BezierPath(const Point2d& start_point,
               const Point2d& end_point,
               const std::vector<Point2d>& control_points,
               const PathInfo_base& path_info,
               float start_s);

    /**
     * @brief Discretize the path to points
     * @param output point list
     */
    virtual void Discretize(StaticPoint2dList& point_list);

    /**
     * @brief Calculate the pose at the distance sr
     * @param s distance of the point(may not be the distance from the start point)
     * @param pose output pose
     */
    virtual void GetPose(float s, Point2d& pose) const;



private:
    std::vector<Vec2d> control_points_;
    float start_theta_;
    float end_theta_;
};

#endif 
