#ifndef PLANNING_PLANNER_BEZIER_PATH_SMOOTHER_H_
#define PLANNING_PLANNER_BEZIER_PATH_SMOOTHER_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "planning/planner/path.h"
#include "planning/planner/spline.h"
/**
 * @class BezierPathSmoother
 *
 * @brief class that smoothes pathes with bezier curve
 */
class BezierPathSmoother
{
public:
  BezierPathSmoother();
  ~BezierPathSmoother();
  virtual bool Smooth(const std::vector<geometry_msgs::Point> &sample_points, 
                      PathList &smoothed_paths);
  void AddSmoothedPath(PathList &smoothed_path_list);
  bool Smooth(double min_radius,
              const std::vector<std::vector<double>> &raw_points,
              std::vector<std::vector<std::vector<double>>> &smooth_points);
  void Discretize(const std::vector<std::vector<double>> &sample_points, const std::vector<double> &X, const std::vector<double> &Y, StaticPoint2dList &point_list);
  double c1_, c2_, c3_, c4_;
  float distance_resolution_;
  inline const StaticPoint2dList &smoothed_path_points() const
  {
    return smoothed_path_info_.points;
  }

  void clearAll() { smoothed_path_info_.clear(); }

private:
  struct PathInfo
  {
    PathInfo() : current_point(points.begin()) {}

    void AddPath(const PathPtr &path);

    inline void clear()
    {
      paths.clear();
      points.clear();
      end_points.clear();
      current_point = points.begin();
    }

    inline int clear_between(const StaticPoint2dIter &begin,
                             const StaticPoint2dIter &end);

    PathList paths;
    StaticPoint2dList points;
    StaticPoint2dIterList end_points;
    StaticPoint2dIter current_point;
  };
  PathInfo smoothed_path_info_;
};

#endif // PLANNING_PLANNER_BEZIER_PATH_SMOOTHER_H_
