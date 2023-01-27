#ifndef PLANNING_GLOBAL_PLANNER_H_
#define PLANNING_GLOBAL_PLANNER_H_

#include <thread>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "planning/planner/path.h"
#include "planning/planner/bezier_path_smoother.h"
// #include "slam_navi_msgs/Path2d.h"
// #include "slam_navi_msgs/Point2d.h"

class BezierPathSmoother;

class GlobalPlanner
{
public:
    GlobalPlanner(uint32_t period_in_ms, ros::NodeHandle prev_nh);

    ~GlobalPlanner();

    bool Init();
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal);
    inline bool start();
    /**
     * @brief Append one navigation task to the list.
     * @param navi_task the new navigation task
     */

    bool GPlanner();
    void CyclicRun();
    double normalized_pi(double alpha);
    void VisualizePath();

private:

    std::vector<geometry_msgs::Point> sample_points_;
    double prev_goal_th_, curr_goal_th_;
    BezierPathSmoother path_smoother_;
    bool sample_finished_;
    uint32_t period_in_ms_; //period in ms
    bool is_running_;
    std::thread thread_;
    StaticPoint2dList smoothed_points_;

    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::Publisher smooth_gpath_publisher_;
    ros::Publisher path_publisher_;
};

#endif // _GLOBAL_PLANNER_H_
