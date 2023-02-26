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

#include "cartesian_planner/cartesian_planner.h"
#include "cartesian_planner/visualization/plot.h"

namespace cartesian_planner {

bool CartesianPlanner::Plan(const StartState &state, DiscretizedTrajectory &result) {
  std::vector<TrajectoryPoint> points;

  nav_msgs::Path reference_path;
  reference_path = env_->getreference_path();

  int min_id = -1;
  float min_dist = FLT_MAX;

  TrajectoryPoint robotPoint;
  robotPoint.x =  state.x;
  robotPoint.y =  state.y;
  for (int i = 0; i < reference_path.poses.size(); ++i)
  {
      TrajectoryPoint p;

      p.x = reference_path.poses[i].pose.position.x;
      p.y = reference_path.poses[i].pose.position.y;
      // p.z = robotPoint.z;
      float dist = pointDistance(p, robotPoint);
      if (dist < min_dist)
      {
          min_dist = dist;
          min_id = i;
      }
  }

  if (min_id >= 0 && min_dist < 1.0)
      reference_path.poses.erase(reference_path.poses.begin(), reference_path.poses.begin() + min_id);
  
  if(reference_path.poses.empty()) {
    ROS_ERROR("reference_path empty");
    return false;
  }

  //max_15m
  int num = config_.nfe;
  double length_max = 15.0 ;
  double lenth = 0.0;
  int end_index = reference_path.poses.size()-1;
  for(int i = 1 ;i<= end_index ;++i)
  {
    lenth +=  hypot(reference_path.poses[i].pose.position.x - reference_path.poses[i-1].pose.position.x, reference_path.poses[i].pose.position.y - reference_path.poses[i-1].pose.position.y);
    if(lenth>=length_max)
    {
        end_index = i;
        break;
    }
  }

        double pathResolution = lenth < length_max ?(lenth/num):(length_max/num);
        double dis = 0, ang = 0;
        float h_z = 0;
        double margin = pathResolution * 0.01;
        double remaining = 0;
        int nPoints = 0;
        nav_msgs::Path fixedPath = reference_path;
        nav_msgs::Path back_z_Path = reference_path;

        fixedPath.poses.clear();
        fixedPath.poses.push_back(reference_path.poses[0]);
        size_t start = 0, next = 1;

        while (next <=end_index)
        {
            dis += hypot(reference_path.poses[next].pose.position.x - reference_path.poses[next-1].pose.position.x, reference_path.poses[next].pose.position.y - reference_path.poses[next-1].pose.position.y) + remaining;
            ang = atan2(reference_path.poses[next].pose.position.y - reference_path.poses[start].pose.position.y, reference_path.poses[next].pose.position.x - reference_path.poses[start].pose.position.x);
            h_z = back_z_Path.poses[next-1].pose.position.z;

            if (dis < pathResolution - margin)
            {
                next++;
                remaining = 0;
            } else if (dis > (pathResolution + margin))
            {
                geometry_msgs::PoseStamped point_start = reference_path.poses[start];
                nPoints = dis / pathResolution;
                for (int j = 0; j < nPoints; j++)
                {
                    point_start.pose.position.x = point_start.pose.position.x + pathResolution * cos(ang);
                    point_start.pose.position.y = point_start.pose.position.y + pathResolution * sin(ang);
                    point_start.pose.position.z = h_z;
                    point_start.pose.orientation = tf::createQuaternionMsgFromYaw(ang);
                    fixedPath.poses.push_back(point_start);
                }
                remaining = dis - nPoints * pathResolution;
                start++;
                reference_path.poses[start].pose.position = point_start.pose.position;
                dis = 0;
                next++;
            } else {
                dis = 0;
                remaining = 0;
                fixedPath.poses.push_back(reference_path.poses[next]);
                next++;
                start = next - 1;
            }
        }

        if(fixedPath.poses.size()>num)
        {
          fixedPath.poses.erase(fixedPath.poses.begin()+num, fixedPath.poses.end());
        }
        else if(fixedPath.poses.size()<num)
        {
            int n = num - fixedPath.poses.size();
            geometry_msgs::PoseStamped point_add = fixedPath.poses[fixedPath.poses.size()-1];
            while (n>0)
            {
              fixedPath.poses.push_back(point_add);
              n--;
            }
            
        }


  

  //等距分割
  if(fixedPath.poses.size()!=num)
  {
    std::cout<<"fixedPath.poses.size()!=num"<<fixedPath.poses.size()<<std::endl;
    return false;
  }

  // config_.nfe

  TrajectoryPoint point;
  for(int i=0;i<fixedPath.poses.size();++i)
  {
    point.x = fixedPath.poses[i].pose.position.x;
    point.y = fixedPath.poses[i].pose.position.y;
    point.z = fixedPath.poses[i].pose.position.z;
    point.theta = tf::getYaw(fixedPath.poses[i].pose.orientation);
    points.emplace_back(point);
  }
  DiscretizedTrajectory coarse_trajectory(points);


  Constraints opti_constraints;
  opti_constraints.start_x = state.x; opti_constraints.start_y = state.y; opti_constraints.start_theta = state.theta;
  opti_constraints.start_v = state.v; opti_constraints.start_phi = state.phi; opti_constraints.start_a = state.a;
  opti_constraints.start_omega = state.omega;

  std::vector<double> coarse_x, coarse_y, coarse_z;
  for(auto &pt: coarse_trajectory.data()) {
    coarse_x.push_back(pt.x); 
    coarse_y.push_back(pt.y);
    coarse_z.push_back(pt.z);

  }

  visualization::Plot(coarse_x, coarse_y, 0.1, visualization::Color::Cyan, 1, "Coarse Trajectory");
  visualization::PlotPoints(coarse_x, coarse_y, 0.3, visualization::Color::Cyan, 2, "Coarse Trajectory");
  visualization::Trigger();

  States optimized;
  if(!opti_.OptimizeIteratively(coarse_trajectory, opti_constraints, optimized)) {
    ROS_ERROR("Optimization failed");
    return false;
  }

  std::vector<double> opti_x, opti_y, opti_v;
  Trajectory result_data;
  double incremental_s = 0.0;
  for(int i = 0; i < config_.nfe; i++) {
    TrajectoryPoint tp;
    incremental_s += i > 0 ? hypot(optimized.x[i] - optimized.x[i-1], optimized.y[i] - optimized.y[i-1]) : 0.0;
    tp.s = incremental_s;

    tp.x = optimized.x[i];
    tp.y = optimized.y[i];
    tp.theta = optimized.theta[i];
    tp.velocity = optimized.v[i];
    tp.kappa = tan(optimized.phi[i]) / config_.vehicle.wheel_base;

    opti_x.push_back(tp.x);
    opti_y.push_back(tp.y);
    opti_v.push_back(tp.velocity);

    result_data.push_back(tp);
  }

  visualization::PlotTrajectory(opti_x, opti_y, opti_v, config_.vehicle.max_velocity, 0.1, visualization::Color::Black, 1, "Optimized Trajectory");
  visualization::Trigger();

  result = DiscretizedTrajectory(result_data);
  return true;
}
}