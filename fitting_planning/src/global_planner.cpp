#include "planning/global_planner.h"

GlobalPlanner::GlobalPlanner(uint32_t period_in_ms, ros::NodeHandle prev_nh) : period_in_ms_(period_in_ms),
                                                                               nh_(prev_nh),
                                                                               sample_finished_(false),
                                                                               is_running_(true)
{
}

GlobalPlanner::~GlobalPlanner() {}

bool GlobalPlanner::Init()
{
    std::cout << "init_ok!" << std::endl;
    //TODO:Initialize the algorigthm modules
    sample_finished_ = false;
    is_running_ = true;
    //TODO: Decide the parameter is set in code or in ros param.
    smooth_gpath_publisher_ = nh_.advertise<geometry_msgs::PoseArray>("gpath/smooth", 1000);
    path_publisher_ = nh_.advertise<nav_msgs::Path>("/path", 1000); //#
    // path_publisher_ = nh_.advertise<slam_navi_msgs::Path2d>("/path", 1000);
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &GlobalPlanner::goalCB, this);
    return true;
}

void GlobalPlanner::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
    if (sample_finished_ == true)
        return;

    sample_points_.push_back(goal->pose.position);
    ROS_INFO("Goal Received :goalCB!");
    if (sample_points_.size() < 2)
    {
        curr_goal_th_ = tf::getYaw(goal->pose.orientation);
        prev_goal_th_ = curr_goal_th_;
    }
    else
    {
        curr_goal_th_ = tf::getYaw(goal->pose.orientation);
        double diff_th = curr_goal_th_ - prev_goal_th_;
        prev_goal_th_ = curr_goal_th_;
        if (fabs(normalized_pi(diff_th)) > 2 * common::kPi / 3)
        {
            smoothed_points_.clear();
            path_smoother_.clearAll();
            sample_finished_ = true;
        }
    }
}

bool GlobalPlanner::GPlanner()
{
    // std::cout << "thread_ok!" << std::endl;
    if (smoothed_points_.size() > 0)
        VisualizePath();
    if (sample_finished_ != true)
    {
        return 0;
    }
    //get path segment
    PathList smooth_paths;
    bool isSucceed = path_smoother_.Smooth(sample_points_,
                                           smooth_paths);
    /*
      according to erery segment to generate smooth_points  
    */
    path_smoother_.AddSmoothedPath(smooth_paths);
    //get interpolated path points
    smoothed_points_ = path_smoother_.smoothed_path_points();
    // std::cout << "path_smoother_isSucceed: " << isSucceed << std::endl;
    // std::cout << "smoothed_points_size: " << smoothed_points_.size() << std::endl;
    VisualizePath();
    if (isSucceed == true)
    {
        sample_points_.clear();
        sample_finished_ = false;
    }
}
inline bool GlobalPlanner::start()
{
    is_running_ = true;
    thread_ = std::thread(std::bind(&GlobalPlanner::CyclicRun, this));

    thread_.detach();
    return true;
}
void GlobalPlanner::CyclicRun()
{
    std::cout << "start run!" << std::endl;
    while (is_running_)
    {
        GPlanner();
        std::this_thread::sleep_for(std::chrono::milliseconds(period_in_ms_));
    }
}

double GlobalPlanner::normalized_pi(double alpha)
{
    if (alpha > common::kPi)
    {
        alpha -= 2 * common::kPi;
    }
    if (alpha < -common::kPi)
    {
        alpha += 2 * common::kPi;
    }
    return alpha;
}

void GlobalPlanner::VisualizePath()
{
    geometry_msgs::PoseArray smooth_gpath;
    // slam_navi_msgs::Path2d wp_path;
    nav_msgs::Path wp_path; //#

    smooth_gpath.header.frame_id = "/map";
    // wp_path.move_base_frame = "/map";

    geometry_msgs::Pose pose;
    // slam_navi_msgs::Point2d waypoint_pose;
    geometry_msgs::PoseStamped waypoint_pose; //#

    ros::Time current_time = ros::Time::now(); //#
    wp_path.header.stamp = current_time;       //#
    wp_path.header.frame_id = "/map";          //#

    static int num = 0;
    int temp_num = smoothed_points_.size();

    if (num < temp_num || true)
    {
        for (const auto &sp : smoothed_points_)
        {
            // waypoint_pose.x = sp.coord()(0);
            // waypoint_pose.y = sp.coord()(1);
            // waypoint_pose.theta = sp.theta();
            // waypoint_pose.curvature = sp.curvature();
            // waypoint_pose.dcurvature = sp.curvature_rate();
            // wp_path.point_list.push_back(waypoint_pose);
            waypoint_pose.pose.position.x = sp.coord()(0);                               //#
            waypoint_pose.pose.position.y = sp.coord()(1);                               //#
            waypoint_pose.pose.orientation = tf::createQuaternionMsgFromYaw(sp.theta()); //#

            wp_path.poses.push_back(waypoint_pose);

            pose.position.x = sp.coord()(0);
            pose.position.y = sp.coord()(1);
            pose.orientation = tf::createQuaternionMsgFromYaw(sp.theta());

            smooth_gpath.poses.push_back(pose);
        }
        num = temp_num;
        smooth_gpath_publisher_.publish(smooth_gpath);
        path_publisher_.publish(wp_path);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fitting_planning_node");
    ros::NodeHandle nh("~");
    GlobalPlanner global_planner(50, nh);

    if (global_planner.Init())
    {
        global_planner.start();
    }
    ros::spin();
    return 0;
}
