/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "cartesian_planner/CenterLine.h"
#include "cartesian_planner/Obstacles.h"
#include "cartesian_planner/DynamicObstacles.h"
#include "cartesian_planner/cartesian_planner.h"

#include "cartesian_planner/visualization/plot.h"




using namespace cartesian_planner;

class CartesianPlannerNode {
public:
  std::mutex mtx;
  nav_msgs::Path globalPathMessage;
  CartesianPlanner::StartState robotstate;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  bool receive = false;

  explicit CartesianPlannerNode(const ros::NodeHandle &nh) : nh_(nh) {
    env_ = std::make_shared<Environment>(config_);
    planner_ = std::make_shared<CartesianPlanner>(config_, env_);

    center_line_subscriber_ = nh_.subscribe("/center_line", 1, &CartesianPlannerNode::CenterLineCallback, this);
    obstacles_subscriber_ = nh_.subscribe("/obstacles", 1, &CartesianPlannerNode::ObstaclesCallback, this);
    dynamic_obstacles_subscriber_ = nh_.subscribe("/dynamic_obstacles", 1,
                                                  &CartesianPlannerNode::DynamicObstaclesCallback, this);//每个时间点对应障碍物的坐标

    goal_subscriber_ = nh_.subscribe("/move_base_simple/goal", 1, &CartesianPlannerNode::PlanCallback, this);

    or_path_subscriber_ = nh_.subscribe("planning/planning/execute_path", 1, &CartesianPlannerNode::Pathcallback, this); //planning/planning/execute_path


    state_.x = 0.0;
    state_.y = 0.0;
    state_.theta = 0.0;
    state_.v = 0.5;
    state_.phi = 0.0;
    state_.a = 0.0;
    state_.omega = 0.0;
  }

  
  inline float NormalizeAngle_pi(const float angle) const
  {
        double v = fmod(angle, 2 * M_PI);  //0~pi
  
          if (v < -M_PI) {
           v += 2.0 * M_PI;
      } else if (v > M_PI) {
         v -= 2.0 * M_PI;
      }

     return v;
  }

  bool getRobotPosition()
  {
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return false; }
        
        robotstate.x = transform.getOrigin().x();
        robotstate.y = transform.getOrigin().y();

        double roll, pitch, yaw;
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(roll, pitch, yaw);
        robotstate.theta = NormalizeAngle_pi(yaw);

        return true;
  }

  void CenterLineCallback(const CenterLineConstPtr &msg) {
    Trajectory data;
    for (auto &pt: msg->points) {
      TrajectoryPoint tp;
      tp.s = pt.s;
      tp.x = pt.x;
      tp.y = pt.y;
      tp.theta = pt.theta;
      tp.kappa = pt.kappa;
      tp.left_bound = pt.left_bound;
      tp.right_bound = pt.right_bound;
      data.push_back(tp);
    }

    env_->SetReference(DiscretizedTrajectory(data));
    env_->Visualize();
  }

  void ObstaclesCallback(const ObstaclesConstPtr &msg) {
    env_->obstacles().clear();
    for (auto &obstacle: msg->obstacles) {
      std::vector<math::Vec2d> points;
      for (auto &pt: obstacle.points) {
        points.emplace_back(pt.x, pt.y);
      }
      env_->obstacles().emplace_back(points);
    }
    env_->Visualize();
  }

  void DynamicObstaclesCallback(const DynamicObstaclesConstPtr &msg) {
    env_->dynamic_obstacles().clear();
    for (auto &obstacle: msg->obstacles) {
      Environment::DynamicObstacle dynamic_obstacle;

      for (auto &tp: obstacle.trajectory) {
        math::Pose coord(tp.x, tp.y, tp.theta);
        std::vector<math::Vec2d> points;
        for (auto &pt: obstacle.polygon.points) {
          points.push_back(coord.transform({pt.x, pt.y, 0.0}));
        }
        math::Polygon2d polygon(points);

        dynamic_obstacle.emplace_back(tp.time, points);
      }

      env_->dynamic_obstacles().push_back(dynamic_obstacle);
    }
    env_->Visualize();
  }

  void PlanCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    // DiscretizedTrajectory result;
    // if(receive)
    // {
    // if (planner_->Plan(state_, result)) {
    //   double dt = config_.tf / (double) (config_.nfe - 1);
    //   for (int i = 0; i < config_.nfe; i++) {
    //     double time = dt * i;
    //     auto dynamic_obstacles = env_->QueryDynamicObstacles(time);
    //     for (auto &obstacle: dynamic_obstacles) {
    //       int hue = int((double) obstacle.first / env_->dynamic_obstacles().size() * 320);

    //       visualization::PlotPolygon(obstacle.second, 0.2, visualization::Color::fromHSV(hue, 1.0, 1.0), obstacle.first,
    //                                  "Online Obstacle");
    //     }

    //     auto &pt = result.data().at(i);
    //     PlotVehicle(1, {pt.x, pt.y, pt.theta}, atan(pt.kappa * config_.vehicle.wheel_base));
    //     ros::Duration(dt).sleep();
    //   }

    //   visualization::Trigger();
    // }
    // }
    // receive = false;
    // std::cout<<"recevee==============================================================fasle"<<std::endl;
  }

double cast_from_0_to_2PI_Angle(const double& ang)
{
    double angle = 0;
    if (ang < -2.0 * M_PI || ang > 2.0 * M_PI) {
        angle = fmod(ang, 2.0 * M_PI);
    } else
        angle = ang;

    if (angle < 0) {
        angle = 2.0 * M_PI + angle;
    }
    return angle;
}

  void Pathcallback(const nav_msgs::Path::ConstPtr& pathMsg) {
      std::lock_guard<std::mutex> lock(mtx); 
      if (pathMsg->poses.size() <= 1)
      {
          ROS_WARN("Empty global path received.");
          return;
      }
      std::cout<<"receive path size"<<pathMsg->poses.size()<<std::endl;
      globalPathMessage = *pathMsg;
      env_->SetReference(globalPathMessage);
      receive = true;

    DiscretizedTrajectory result;
    if(receive)
    {
    if (planner_->Plan(state_, result)) {
      double dt = config_.tf / (double) (config_.nfe - 1);
      for (int i = 0; i < config_.nfe; i++) {
        double time = dt * i;
        auto dynamic_obstacles = env_->QueryDynamicObstacles(time);
        for (auto &obstacle: dynamic_obstacles) {
          int hue = int((double) obstacle.first / env_->dynamic_obstacles().size() * 320);

          visualization::PlotPolygon(obstacle.second, 0.2, visualization::Color::fromHSV(hue, 1.0, 1.0), obstacle.first,
                                     "Online Obstacle");
        }

        auto &pt = result.data().at(i);
        PlotVehicle(1, {pt.x, pt.y, pt.theta}, atan(pt.kappa * config_.vehicle.wheel_base));
        ros::Duration(dt).sleep();
      }

      visualization::Trigger();
    }
    }
    receive = false;
  }

private:
  ros::NodeHandle nh_;
  cartesian_planner::CartesianPlannerConfig config_;
  Env env_;
  std::shared_ptr<cartesian_planner::CartesianPlanner> planner_;
  CartesianPlanner::StartState state_;

  ros::Subscriber center_line_subscriber_, obstacles_subscriber_, dynamic_obstacles_subscriber_, goal_subscriber_,or_path_subscriber_;

  void PlotVehicle(int id, const math::Pose &pt, double phi) {
    auto tires = GenerateTireBoxes({pt.x(), pt.y(), pt.theta()}, phi);

    int tire_id = 1;
    for (auto &tire: tires) {
      visualization::PlotPolygon(math::Polygon2d(tire), 0.1, visualization::Color::White, id * (tire_id++),
                                 "Tires");
    }
    visualization::PlotPolygon(math::Polygon2d(config_.vehicle.GenerateBox({pt.x(), pt.y(), pt.theta()})), 0.2,
                               visualization::Color::Yellow, id, "Footprint");
    visualization::Trigger();
  }

  std::array<math::Box2d, 4> GenerateTireBoxes(const math::Pose pose, double phi = 0.0) const {
    auto front_pose = pose.extend(config_.vehicle.wheel_base);
    auto track_width = config_.vehicle.width - 0.195;
    double rear_track_width_2 = track_width / 2, front_track_width_2 = track_width / 2;
    double box_length = 0.6345;
    double sin_t = sin(pose.theta());
    double cos_t = cos(pose.theta());
    return {
      math::Box2d({pose.x() - rear_track_width_2 * sin_t, pose.y() + rear_track_width_2 * cos_t}, pose.theta(),
                  box_length, 0.195),
      math::Box2d({pose.x() + rear_track_width_2 * sin_t, pose.y() - rear_track_width_2 * cos_t}, pose.theta(),
                  box_length, 0.195),
      math::Box2d({front_pose.x() - front_track_width_2 * sin_t, front_pose.y() + front_track_width_2 * cos_t},
                  front_pose.theta() + phi, box_length, 0.195),
      math::Box2d({front_pose.x() + front_track_width_2 * sin_t, front_pose.y() - front_track_width_2 * cos_t},
                  front_pose.theta() + phi, box_length, 0.195),
    };
  }
};
// 35 -15
int main(int argc, char **argv) {
  ros::init(argc, argv, "cartesian_planner_node");

  ros::NodeHandle nh;
  visualization::Init(nh, "map", "cartesian_planner_markers");

  CartesianPlannerNode node(nh);
  ros::spin();
  return 0;
}