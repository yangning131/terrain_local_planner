#ifndef TRAJECTORY_CONTROL_REPLANNING_TRACKING_GEO_2D_H_
#define TRAJECTORY_CONTROL_REPLANNING_TRACKING_GEO_2D_H_

#include <list>
#include <iostream>
#include <Eigen/Dense>
#include "planning/common/stdandard_def.h"
#include "planning/common/basic_math.h"
#include <string>
#include <vector>
using namespace std;

typedef Eigen::Vector2f Vec2d;
const float kDistanceEps = 0.0001;
const float kAngleEps = 0.0001;

/**
 * @struct Point2d
 * @brief 2D point struct
 */
struct Point2d {
    Point2d() : x(0.0), y(0.0), theta(0.0) {}

    Point2d(float x, float y, float theta) : x(x), y(y), theta(theta) {}

    float x;
    float y;
    float theta;
};

/**
 * @struct PathInfo_base
 * @brief struct containing path identification and other information.
 */
struct PathInfo_base {
    PathInfo_base(uint32_t id, bool is_ignore_end_pose, bool is_forward, const std::string& frame) :
        id(id), is_ignore_end_pose(is_ignore_end_pose), is_forward(is_forward), move_base_frame(frame) {}

    uint32_t id;
    bool is_ignore_end_pose;        
    bool is_forward;                //Is the move if forward or backward
    std::string move_base_frame;    //the frame of the path coordinates
};

/**
 * @class StaticPath2dInfo
 * @brief Class including the static point information: x, y, tangent vector,
 *      normal vector, curvature and distance.
 */
class StaticPoint2dInfo {
public:
    StaticPoint2dInfo(const Vec2d& coord, const Vec2d& tangent, float curvature, float curvature_rate,
                      float distance, float theta, const PathInfo_base& path_info) :
        coord_(coord), tangent_(tangent), curvature_(curvature), curvature_rate_(curvature_rate),
        distance_(distance), theta_(theta), path_info_(path_info),
        max_linear_acc_(FLOAT_INF),
        max_linear_dec_(FLOAT_INF),
        max_linear_speed_(FLOAT_INF),
        max_angular_acc_(FLOAT_INF),
        max_angular_dec_(FLOAT_INF),
        max_angular_speed_(FLOAT_INF) {}

    StaticPoint2dInfo(const StaticPoint2dInfo& point, float distance, const PathInfo_base& path_info) :
        coord_(point.coord_), tangent_(point.tangent_), curvature_(point.curvature_),
        curvature_rate_(point.curvature_rate_), distance_(distance), theta_(point.theta_), path_info_(path_info),
        max_linear_acc_(point.max_linear_acc_),
        max_linear_dec_(point.max_linear_dec_),
        max_linear_speed_(point.max_linear_speed_),
        max_angular_acc_(point.max_angular_acc_),
        max_angular_dec_(point.max_angular_dec_),
        max_angular_speed_(point.max_angular_speed_) {}

    /**
     * @brief append one point after this point, change the curvature rate
     */
    inline void connect_point(const StaticPoint2dInfo& prev_point, StaticPoint2dInfo& next_point) {
        //Check whether the position of the two points is the same. If they are the same, the angle, curvature should
        //be all same, or the curvature is infinite and so is the curvature rate
        if(next_point.distance_ == distance_) {
            if(ABS(next_point.theta_ - theta_) > kAngleEps) {
                curvature_ = (next_point.theta_ - theta_) / 0.0;
                curvature_rate_ = -curvature_;
                next_point.curvature_ = curvature_;
                next_point.curvature_rate_ = curvature_rate_;
            }
            else {
                if(next_point.curvature_ == curvature_) {
                    if(prev_point.curvature_ == curvature_) {
                        curvature_rate_ = 0.0;
                        next_point.curvature_rate_ = 0.0;
                    }
                    else {
                        curvature_rate_ = (curvature_ - prev_point.curvature_) / (distance_ - prev_point.distance_);
                        next_point.curvature_rate_ = curvature_rate_;
                    }
                }
                else {
                    curvature_rate_ = (next_point.curvature_ - curvature_) / 0.0;
                    next_point.curvature_rate_ = curvature_rate_;
                }
            }
        }
        else {
            float delta_s = next_point.distance_ - distance_;
            next_point.curvature_ = (next_point.theta_ - theta_) / delta_s;
            next_point.curvature_rate_ = (next_point.curvature_ - curvature_) / delta_s;
            curvature_rate_ = (curvature_ - prev_point.curvature_) / (distance_ - prev_point.distance_);
        }
    }

    inline float manhattan_distance(const Point2d& pose) const {
        return ABS(pose.x - coord_[0]) + ABS(pose.y - coord_[1]);
    }

    inline float theta_distance(const Point2d& pose) const {
        return ABS(pose.theta - theta());
    }

    inline float x() const {
        return coord_[0];
    }

    inline float y() const {
        return coord_[1];
    }

    inline const Vec2d& coord() const {
        return coord_;
    }

    inline const Vec2d& tangent() const {
        return tangent_;
    }

    inline Vec2d normal() const {
        return Vec2d(-tangent_[1], tangent_[0]);
    }

    inline float curvature() const {
        return curvature_;
    }

    inline float curvature_rate() const {
        return curvature_rate_;
    }

    inline float distance() const {
        return distance_;
    }

    inline float theta() const {
        return theta_;
    }

    inline const PathInfo_base& path_info() const {
        return path_info_;
    }

    inline void set_distance(float distance) {
        distance_ = distance;
    }

    inline float max_linear_acc() const {
        return max_linear_acc_;
    }

    inline void set_max_linear_acc(float max_linear_acc) {
        max_linear_acc_ = max_linear_acc;
    }

    inline float max_linear_dec() const {
        return max_linear_dec_;
    }

    inline void set_max_linear_dec(float max_linear_dec) {
        max_linear_dec_ = max_linear_dec;
    }

    inline float max_linear_speed() const {
        return max_linear_speed_;
    }

    inline void set_max_linear_speed(float max_linear_speed) {
        max_linear_speed_ = max_linear_speed;
    }

    inline float max_angular_speed() const {
        return max_angular_speed_;
    }

    inline void set_max_angular_speed(float max_angular_speed) {
        max_angular_speed_ = max_angular_speed;
    }

    inline float max_angular_acc() const {
        return max_angular_acc_;
    }

    inline void set_max_angular_acc(float max_angular_acc) {
        max_angular_acc_ = max_angular_acc;
    }

    inline float max_angular_dec() const {
        return max_angular_dec_;
    }

    inline float set_max_angular_dec(float max_angular_dec) {
        max_angular_dec_ = max_angular_dec;
    }

    /**
     * @brief Check the given coordinate (x,y) is behind this point
     */
    inline bool is_behind(const Point2d& pose) const {
        if(std::isfinite(curvature_)) {
            Vec2d p(pose.x, pose.y);
            Vec2d v(p - coord_);
            float dir = v.dot(tangent_);
            if(dir > 0.0) {
                return true;
            }
            return false;
        }
        //current point is a rotation pose, check the rotation direction and judge (x, y) is in front of the point
        //or not
        bool is_postive_inf = (curvature_ > 0.0);
        if(is_postive_inf ^ (pose.theta > theta_)) {
            return true;
        }
        return false;
    }

    inline double dist(const Point2d& pose) const {//
        Vec2d p(pose.x, pose.y);
        Vec2d v(p - coord_);
        return sqrt(v(0) * v(0) + v(1) * v(1));
    }

    /**
     * @brief Check whether the given coordinate (x,y) is in front of this point
     */
    inline bool is_in_front(const Point2d& pose) {
        return !is_behind(pose);
    }
private:
    Vec2d coord_;
    Vec2d tangent_;

    float curvature_;
    float curvature_rate_;
    float distance_;
    float theta_;

    PathInfo_base path_info_;

    float max_linear_acc_;
    float max_linear_dec_;
    float max_linear_speed_;

    float max_angular_speed_;
    float max_angular_acc_;
    float max_angular_dec_;
};

/**
 * @class DynamicPoint2dInfo
 *
 * @brief Class including the static point information and motion parameters, including allowed maximum linear speed
 *      on this point.
 */
class DynamicPoint2dInfo : public StaticPoint2dInfo {
public:

    DynamicPoint2dInfo(const StaticPoint2dInfo& static_point_info) :
        StaticPoint2dInfo(static_point_info),
        dyn_max_linear_acc_(static_point_info.max_linear_acc()),
        dyn_max_linear_dec_(static_point_info.max_linear_dec()),
        dyn_max_linear_speed_(static_point_info.max_linear_speed()),
        dyn_max_angular_acc_(static_point_info.max_angular_acc()),
        dyn_max_angular_dec_(static_point_info.max_angular_dec()),
        dyn_max_angular_speed_(static_point_info.max_angular_speed()) {}

    DynamicPoint2dInfo(const StaticPoint2dInfo& static_point_info,
                       float distance) :
        StaticPoint2dInfo(static_point_info),
        dyn_max_linear_acc_(static_point_info.max_linear_acc()),
        dyn_max_linear_dec_(static_point_info.max_linear_dec()),
        dyn_max_linear_speed_(static_point_info.max_linear_speed()),
        dyn_max_angular_acc_(static_point_info.max_angular_acc()),
        dyn_max_angular_dec_(static_point_info.max_angular_dec()),
        dyn_max_angular_speed_(static_point_info.max_angular_speed()) {
        set_distance(distance);
    }

    inline float dyn_max_linear_acc() const {
        return dyn_max_linear_acc_;
    }

    inline void set_dyn_max_linear_acc(float dyn_max_linear_acc) {
        dyn_max_linear_acc_ = dyn_max_linear_acc;
    }

    inline float dyn_max_linear_dec() const {
        return dyn_max_linear_dec_;
    }

    inline void set_dyn_max_linear_dec(float dyn_max_linear_dec) {
        dyn_max_linear_dec_ = dyn_max_linear_dec;
    }

    inline float dyn_max_linear_speed() const {
        return dyn_max_linear_speed_;
    }

    inline void set_dyn_max_linear_speed(float dyn_max_linear_speed) {
        dyn_max_linear_speed_ = dyn_max_linear_speed;
    }

    inline float dyn_max_angular_speed() const {
        return dyn_max_angular_speed_;
    }

    inline void set_dyn_max_angular_speed(float dyn_max_angular_speed) {
        dyn_max_angular_speed_ = dyn_max_angular_speed;
    }

    inline float dyn_max_angular_acc() const {
        return dyn_max_angular_acc_;
    }

    inline void set_dyn_max_angular_acc(float dyn_max_angular_acc) {
        dyn_max_angular_acc_ = dyn_max_angular_acc;
    }

    inline float dyn_max_angular_dec() const {
        return dyn_max_angular_dec_;
    }

    inline float set_dyn_max_angular_dec(float dyn_max_angular_dec) {
        dyn_max_angular_dec_ = dyn_max_angular_dec;
    }

private:
    float dyn_max_linear_acc_;
    float dyn_max_linear_dec_;
    float dyn_max_linear_speed_;

    float dyn_max_angular_speed_;
    float dyn_max_angular_acc_;
    float dyn_max_angular_dec_;
};

typedef std::list<StaticPoint2dInfo> StaticPoint2dList;
typedef std::list<std::list<StaticPoint2dInfo>::iterator> StaticPoint2dIterList;
typedef std::list<StaticPoint2dInfo>::iterator StaticPoint2dIter;
typedef std::list<StaticPoint2dInfo>::const_iterator ConstStaticPoint2dIter;
typedef std::vector<DynamicPoint2dInfo> DynamicPoint2dVector;
typedef std::vector<DynamicPoint2dInfo>::iterator DynamicPoint2dIter;
typedef std::vector<DynamicPoint2dInfo>::const_iterator ConstDynamicPoint2dIter;

/**
 * @brief find the point mapped to the given point on the point list.
 */
inline ConstStaticPoint2dIter find_mapped_point(const StaticPoint2dList& point_list,
                                                const Point2d& point_to_be_mapped) {
    for(ConstStaticPoint2dIter iter = point_list.begin(); iter != point_list.end(); ++iter) {
        if((*iter).is_behind(point_to_be_mapped)) {
            iter++;
        }
        else {
            return iter;
        }
    }
    return point_list.begin();
}

#endif 
