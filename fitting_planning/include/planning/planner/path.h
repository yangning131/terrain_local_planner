#ifndef PLANNING_PLANNER_PATH_PATH_H_
#define PLANNING_PLANNER_PATH_PATH_H_

#include <list>
#include <boost/shared_array.hpp>
#include "planning/planner/geo_2d.h"

/**
 * @class Path
 *
 * @brief The path interface class
 */

class Path
{
public:
    Path(const Point2d &start_point,
         const Point2d &end_point,
         const PathInfo_base &path_info,
         float start_s = 0.0) : start_point_(start_point.x, start_point.y),
                                end_point_(end_point.x, end_point.y),
                                start_s_(start_s),
                                end_s_(start_s),
                                length_(0.0),
                                path_info_(path_info) {}

    /**
     * @brief Discretize the path to points
     * @param output point list
     */
    virtual void Discretize(StaticPoint2dList &point_list) = 0;

    static void set_distance_resolution(float resolution)
    {
        distance_resolution_ = resolution;
    }

    static void set_theta_resoltion(float resolution)
    {
        angular_resolution_ = resolution;
    }

    /**
     * @brief Calculate the pose at the distance s
     * @param s distance of the point(may not be the distance from the start point)
     * @param pose output pose
     */
    virtual void GetPose(float s, Point2d &pose) const = 0;

    inline float start_s() const
    {
        return start_s_;
    }

    inline void set_start_s(float start_s)
    {
        start_s_ = start_s;
        end_s_ = start_s + length_;
    }

    inline float end_s() const
    {
        return end_s_;
    }

    inline float length() const
    {
        return length_;
    }

    inline const PathInfo_base &path_info() const
    {
        return path_info_;
    }

    inline Vec2d start_point() const
    {
        return start_point_;
    }

    inline Vec2d end_point()
    {
        return end_point_;
    }

protected:
    static float distance_resolution_;
    static float angular_resolution_;

    Vec2d start_point_;
    Vec2d end_point_;
    float start_s_; //distance value at the start point
    float end_s_;   //distance value at the end point
    float length_;  //length of the path

    PathInfo_base path_info_; //struct containing the describtion path information
};

typedef boost::shared_ptr<Path> PathPtr;
typedef std::list<PathPtr> PathList;
typedef std::list<PathPtr>::iterator PathListIter;
typedef std::list<PathPtr>::const_iterator ConstPathListIter;

#endif
