#include <kdtree1/kdtree.h>
//#include "planner.h"
#include "constants.h"
#include "node3d.h"
#include "visualize.h"
#include "path.h"
#include "collisiondetection.h"
#include <boost/heap/binomial_heap.hpp>
#include "utility.h"



#include "state_node.h"
// #include "utility.h"
#include "timer.h"
#include <glog/logging.h>

#include <map>
#include <memory>
#include "rs_path.h"
#include "type.h"



using namespace cv;
//using namespace VisualizeNS;
class PathPlanning : public ParamServer
{
   public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::mutex mtx;
    ros::Timer pathUpdateTimer;
    ros::Timer pathPublishTimer;

    ros::Subscriber subGlobalPath;
    ros::Subscriber subObstacleMap;
    ros::Subscriber subPathMap;

    ros::Subscriber subImage_collision;
    ros::Subscriber subImage_cost;
    ros::Subscriber subImage_pathmap;
    ros::Subscriber subImage_pathmapBound;
    ros::Publisher  pubImage_robot;

    ros::Publisher  pubImage_show;

    ros::Publisher pubExecutePath;
    ros::Publisher pubSearchedPath;
    ros::Publisher pubPRMGraph;
    ros::Publisher pubSingleSourcePaths;

    ros::Publisher pub_delete_openplanner;

    nav_msgs::OccupancyGrid occupancyMap2D;
    nav_msgs::OccupancyGrid Path_Map2D;

    PointType robotPoint;
    PointType fixedPoint;

    PointType lastRobotPoint;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    vector<state_t*> nodeList;
    vector<PointType*>  obtest;
    vector<vector<state_t*> > adjacencyMatrix;

    int adjacency_width_grid;
    int adjacency_length_grid;

    nav_msgs::Path globalPathMessage;
    nav_msgs::Path executePath;
    nav_msgs::Path globalPath;
    nav_msgs::Path centerPath;
    nav_msgs::Path remainingPath;
    nav_msgs::Path searchedPath;
    vector<nav_msgs::Path> alternativePaths;
    
    Node3D nStart4;
    //HybridAStar::Planner hy;
    Visualize visualization_;
    //int   visualization;
    CollisionDetection configurationSpace;
    Constants::config collisionLookup[Constants::headings * Constants::positions]; 

    kdtree* kd_tree_;

    Path path;
   

    RolloutGeneratorNS::RolloutGenerator openPlannerRollOut;

    InitChassisDescriptor Chassis;
    cv::Mat ImageMap;
    cv::Mat ImageMapshow;
    bool receive_imagemap = false;
    cv::Mat ImageMapCost;

    cv::Mat ImagePathMap;
    cv::Mat ImagePathMapBound;

    /*Hybrid*/

    ros::Publisher searched_tree_pub_;
    ros::Publisher vehicle_path_pub_;
    ros::Publisher path_pub_;
    

    /*Hybrid*/
    private:
    uint8_t *map_data_ = nullptr;
    double STATE_GRID_RESOLUTION_{};
    double ANGULAR_RESOLUTION_{};
    int STATE_GRID_SIZE_X_{}, STATE_GRID_SIZE_Y_{}, STATE_GRID_SIZE_PHI_{};
    int MAP_GRID_SIZE_X_{}, MAP_GRID_SIZE_Y_{};

    double map_x_lower_{}, map_x_upper_{}, map_y_lower_{}, map_y_upper_{};

    StateNode::Ptr terminal_node_ptr_ = nullptr;
    StateNode::Ptr ***state_node_map_ = nullptr;

    std::multimap<double, StateNode::Ptr> openset_;

    double wheel_base_; //The distance between the front and rear axles
    double segment_length_;
    double move_step_size_;
    double steering_radian_step_size_;
    double steering_radian_; //radian
    double tie_breaker_;

    double shot_distance_;
    int segment_length_discrete_num_;
    int steering_discrete_num_;
    double steering_penalty_;
    double reversing_penalty_;
    double steering_change_penalty_;

    double path_length_ = 0.0;

    std::shared_ptr<RSPath> rs_path_ptr_;

    //VecXd vehicle_shape_;
    //MatXd vehicle_shape_discrete_;

    // debug
    double check_collision_use_time = 0.0;
    int num_check_collision = 0;
    int visited_node_number_ = 0;


    //****///
     std::vector<Node3D> path_node;
 public:
        PathPlanning()
    {   
        subGlobalPath = nh.subscribe<nav_msgs::Path>("expath222", 5, &PathPlanning::pathHandler, this);
        subObstacleMap = nh.subscribe<nav_msgs::OccupancyGrid>("planning/obstacle/map_inflated", 5, &PathPlanning::mapHandler, this);

        subPathMap = nh.subscribe<nav_msgs::OccupancyGrid>("planning/pathmap_path", 5, &PathPlanning::path_mapHandler, this);

        subImage_collision = nh.subscribe<sensor_msgs::Image> ("planning/obstacle/ImageMap", 1,&PathPlanning::ImageMapCallback, this);
        subImage_cost = nh.subscribe<sensor_msgs::Image> ("planning/obstacle/ImageMapCost", 1,&PathPlanning::ImageMapCostCallback, this);

        subImage_pathmap = nh.subscribe<sensor_msgs::Image> ("planning/Image_pathmap", 1,&PathPlanning::ImagePathMapCallback, this);
        subImage_pathmapBound = nh.subscribe<sensor_msgs::Image> ("planning/Image_pathmapBound", 1,&PathPlanning::ImagePathMapBoundCallback, this);
        


        pubPRMGraph = nh.advertise<visualization_msgs::MarkerArray>("planning/planning/prm_graph", 5);
        pubSingleSourcePaths = nh.advertise<visualization_msgs::MarkerArray>("planning/planning/prm_single_source_paths", 5);

        pub_delete_openplanner = nh.advertise<visualization_msgs::MarkerArray>("planning/planning/open_planner", 5);

        pubExecutePath = nh.advertise<nav_msgs::Path>("planning/planning/execute_path222", 1);
        pubSearchedPath = nh.advertise<nav_msgs::Path>("planning/planning/searched_path", 1);

        pubImage_robot  = nh.advertise<sensor_msgs::Image>("planning/obstacle/Imagerobot", 1);

        pubImage_show  = nh.advertise<sensor_msgs::Image>("planning/planning/Image_show", 1);



        /* Hybrid */
        double steering_angle = nh.param("planner/steering_angle", 30);
        int steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 3);
        double wheel_base = nh.param("planner/wheel_base", 1.0);
        double segment_length = nh.param("planner/segment_length", 1.6);
        int segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8);
        double steering_penalty = nh.param("planner/steering_penalty", 1.5);
        double steering_change_penalty = nh.param("planner/steering_change_penalty", 2.0);
        double reversing_penalty = nh.param("planner/reversing_penalty", 3.0);
        double shot_distance = nh.param("planner/shot_distance", 5.0);

        HybridAStar_construct(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance ,72);

        hybrid_Init();

        // path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);

        path_pub_ = nh.advertise<nav_msgs::Path>("planning/planning/execute_path", 1);

        searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
        vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);

        /* Hybrid */

        ImageMap.release();
        ImageMap = cv::Mat::zeros(400, 400, CV_8UC1);

        ImageMapCost.release();
        ImageMapCost = cv::Mat::zeros(400, 400, CV_8UC1);

        ImagePathMap.release();
        ImagePathMap = cv::Mat::zeros(400, 400, CV_8UC1);

        ImagePathMapBound.release();
        // ImagePathMapBound = cv::Mat::zeros(103, 103, CV_8UC1);

        adjacency_width_grid = -1;
        adjacency_length_grid = -1;

        pathUpdateTimer = nh.createTimer(ros::Duration(0.4), &PathPlanning::updatePath, this);
        // pathPublishTimer = nh.createTimer(ros::Duration(0.05), &PathPlanning::publishPath, this);

    }

     const std::vector<Node3D>& getPath() {return path_node;}

        inline float NormalizeAngle(const float angle) const
    {

        float res = angle;  //0~2pi
        while (res >= _2PI_)
        {
            res -= _2PI_;
            //res = res%procConfig_.numAngleStep;
        }
        while (res < 0)
        {
            res += _2PI_;
        }
        return res;
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
    
    void HybridAStar_construct(double steering_angle, int steering_angle_discrete_num, double segment_length,
                         int segment_length_discrete_num, double wheel_base, double steering_penalty,
                         double reversing_penalty, double steering_change_penalty, double shot_distance,
                         int grid_size_phi) {
    wheel_base_ = wheel_base;
    segment_length_ = segment_length;
    steering_radian_ = steering_angle * M_PI / 180.0; // angle to radian
    steering_discrete_num_ = steering_angle_discrete_num;
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;
    move_step_size_ = segment_length / segment_length_discrete_num;
    segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num);
    steering_penalty_ = steering_penalty;
    steering_change_penalty_ = steering_change_penalty;
    reversing_penalty_ = reversing_penalty;
    shot_distance_ = shot_distance;

    CHECK_EQ(static_cast<float>(segment_length_discrete_num_ * move_step_size_), static_cast<float>(segment_length_))
        << "The segment length must be divisible by the step size. segment_length: "
        << segment_length_ << " | step_size: " << move_step_size_;

    rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_));
    tie_breaker_ = 1.0 + 1e-3;

    STATE_GRID_SIZE_PHI_ = grid_size_phi;
    ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI / 180.0;
}

void hybrid_Init() {
    // SetVehicleShape(4.7, 0.3, 1.3);//长  宽  到后边距离

    STATE_GRID_SIZE_X_ = _local_map_grid_num;
    STATE_GRID_SIZE_Y_ = _local_map_grid_num;

    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {

            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    state_node_map_ = new StateNode::Ptr **[STATE_GRID_SIZE_X_];
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        state_node_map_[i] = new StateNode::Ptr *[STATE_GRID_SIZE_Y_];
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            state_node_map_[i][j] = new StateNode::Ptr[STATE_GRID_SIZE_PHI_];
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                state_node_map_[i][j][k] = nullptr;
            }
        }
    }
}

void Hybrid_Reset() {
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
            }
        }
    }

    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
}

inline Vec_3i State2Index(const Vect_3d &state) const {
    Vec_3i index;

    index[0] = std::min(std::max(int((state[0] - map_x_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_X_ - 1);
    index[1] = std::min(std::max(int((state[1] - map_y_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);
    index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);

    return index;
}

double ComputeH(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &terminal_node_ptr) {
    double h;
    // L2
    //    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).norm();

    // L1
    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).lpNorm<1>();

 
    if (h < 3.0 * shot_distance_) {
  
        h = rs_path_ptr_->Distance(current_node_ptr->state_.x(), current_node_ptr->state_.y(),
                                   current_node_ptr->state_.z(),
                                   terminal_node_ptr->state_.x(), terminal_node_ptr->state_.y(),
                                   terminal_node_ptr->state_.z());

    }

    return h;
}

bool AnalyticExpansions(const StateNode::Ptr &current_node_ptr,
                                     const StateNode::Ptr &goal_node_ptr, double &length) {
    VectorVec3d rs_path_poses = rs_path_ptr_->GetRSPath(current_node_ptr->state_,
                                                        goal_node_ptr->state_,
                                                        move_step_size_, length);
   
    for (const auto &pose: rs_path_poses)
    {   
       // std::cout<<"pose yaw = "<<pose.z()<<std::endl;
        // if (isBeyondBoundary_Byimage((float)pose.x(), (float)pose.y(), (float)pose.z())||isIncollision_Byimage((float)pose.x(), (float)pose.y(), (float)pose.z())) {
        //     return false;
        // }   

               if (isBeyondBoundary_Byimage(pose.head(2))||isIncollision_Byimage((float)pose.x(), (float)pose.y(), (float)pose.z())) {
            return false;
        }
       
    }

    goal_node_ptr->intermediate_states_ = rs_path_poses;
    goal_node_ptr->parent_node_ = current_node_ptr;

    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);

    return true;
}

void DynamicModel(const double &step_size, const double &phi,
                               double &x, double &y, double &theta) const {
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);
    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));//Mod2Pi  static
}

void DynamicModel_side(const double &step_size, const int &i,
                               double &x, double &y, double &theta) const {
    // x = x + step_size * std::cos(theta);
    // y = y + step_size * std::sin(theta);
    // theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));//Mod2Pi  static
    x = x + i*step_size * std::sin(theta);
    y = y + i*step_size * std::cos(theta);
}

static double Mod2Pi(const double &x)  {
    double v = fmod(x, 2 * M_PI);
  
      if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

void GetNeighborNodes(const StateNode::Ptr &curr_node_ptr,
                                   std::vector<StateNode::Ptr> &neighbor_nodes) {
    neighbor_nodes.clear();

    for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++i) {//-1 0 1
        VectorVec3d intermediate_state;
        bool has_obstacle = false;

        double x = curr_node_ptr->state_.x();
        double y = curr_node_ptr->state_.y();
        double theta = curr_node_ptr->state_.z();

        const double phi = i * steering_radian_step_size_;  //steering_radian_ / steering_discrete_num_;

        // forward
        for (int j = 1; j <= segment_length_discrete_num_; j++) {//segment_length_discrete_num_ :8  length :1.6   move_step_size_:1.6/8
     
            DynamicModel(move_step_size_, phi, x, y, theta);
                
            intermediate_state.emplace_back(Vect_3d(x, y, theta));
            
             
            if (isIncollision_Byimage(x, y, theta)) {//have opencv
            
                has_obstacle = true;
                break;
            }
            
        }
     
        Vec_3i grid_index = State2Index(intermediate_state.back());//  intermediate_state.back().x(), intermediate_state.back().y(), intermediate_state.back().z()
      
        if (!isBeyondBoundary_Byimage(intermediate_state.back().head(2)) && !has_obstacle) {
          
            auto neighbor_forward_node_ptr = new StateNode(grid_index);
            neighbor_forward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_forward_node_ptr->state_ = intermediate_state.back();
            neighbor_forward_node_ptr->steering_grade_ = i;
            neighbor_forward_node_ptr->direction_ = StateNode::FORWARD;
            neighbor_nodes.push_back(neighbor_forward_node_ptr);
        }

        /*sidewardhaha*/

    if(use_sidemodel)
    {

        if(i!=0)
        {   
            has_obstacle = false;
            intermediate_state.clear();
            x = curr_node_ptr->state_.x();
            y = curr_node_ptr->state_.y();
            theta = curr_node_ptr->state_.z();
            for (int j = 1; j <= segment_length_discrete_num_; j++) {//segment_length_discrete_num_ :8  length :1.6   move_step_size_:1.6/8
            
            DynamicModel_side(move_step_size_, i, x, y, theta);
                
            intermediate_state.emplace_back(Vect_3d(x, y, theta));
            
             
            if (isIncollision_Byimage(x, y, theta)) {//have opencv
            
                has_obstacle = true;
                break;
            }
            
         }
     
         grid_index = State2Index(intermediate_state.back());//  intermediate_state.back().x(), intermediate_state.back().y(), intermediate_state.back().z()
      
         if (!isBeyondBoundary_Byimage(intermediate_state.back().head(2)) && !has_obstacle) {
          
            auto neighbor_backward_node_ptr = new StateNode(grid_index);
            neighbor_backward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_backward_node_ptr->state_ = intermediate_state.back();
            neighbor_backward_node_ptr->steering_grade_ = 0;
            neighbor_backward_node_ptr->direction_ = StateNode::BACKWARD;
            neighbor_nodes.push_back(neighbor_backward_node_ptr);
         }
       
        }/*sideward*/
    }
    else
    {   
        if(use_backwardmodel)
        {
        // backward
        has_obstacle = false;
        intermediate_state.clear();
        x = curr_node_ptr->state_.x();
        y = curr_node_ptr->state_.y();
        theta = curr_node_ptr->state_.z();
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(-move_step_size_, phi, x, y, theta);
          
            intermediate_state.emplace_back(Vect_3d(x, y, theta));

            if (isIncollision_Byimage(x, y, theta)) {
               
                has_obstacle = true;
                break;
            }
        }
       
        if (!isBeyondBoundary_Byimage(intermediate_state.back().head(2)) && !has_obstacle) {
            grid_index = State2Index(intermediate_state.back());
            auto neighbor_backward_node_ptr = new StateNode(grid_index);
            neighbor_backward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_backward_node_ptr->state_ = intermediate_state.back();
            neighbor_backward_node_ptr->steering_grade_ = i;
            neighbor_backward_node_ptr->direction_ = StateNode::BACKWARD;
            neighbor_nodes.push_back(neighbor_backward_node_ptr);
           
        }
        }
       
    }
    }
}

double ComputeG(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &neighbor_node_ptr) const {
    double g;
    if (neighbor_node_ptr->direction_ == StateNode::FORWARD) {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_;
            } else {
                g = segment_length_ * steering_penalty_;
            }
        }
    } else {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_penalty_ * reversing_penalty_;
            }
        }
    }

    return g;
}


VectorVec3d GetPath() const {
    VectorVec3d path;

    std::vector<StateNode::Ptr> temp_nodes;

    StateNode::Ptr state_grid_node_ptr = terminal_node_ptr_;
    while (state_grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr->parent_node_;
    }

    std::reverse(temp_nodes.begin(), temp_nodes.end());
    for (const auto &node: temp_nodes) {
        path.insert(path.end(), node->intermediate_states_.begin(),
                    node->intermediate_states_.end());
    }

    return path;
}

void save_KinoPath(const VectorVec3d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.15;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "map";
    nav_path.header.stamp = ros::Time::now();
    // executePath = nav_path;



    getlowobposition(nav_path);
    executePath = nav_path;

    path_pub_.publish(nav_path);
}

void getlowobposition(nav_msgs::Path &nav_path)
{
    for (int i = 0; i < nav_path.poses.size()-4; ++i)
        {
            float x = nav_path.poses[i].pose.position.x;
            float y = nav_path.poses[i].pose.position.y;
            float yaw = tf::getYaw(nav_path.poses[i].pose.orientation);

            if (findlowobpose_Byimage(x, y, yaw))// || isCloseCollision(x, y))
            {
                nav_path.poses[i].pose.position.z=0.2;
            } 
        }
}

void Publish_imageshow(const VectorVec3d &path) {

    cv::Mat robot_thata;
    cv::Mat imageROI_show;
    cv::Mat ImageMap_show = ImageMap.clone();
    cv::Mat ImageMap_show_ = ImageMap.clone();
    

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {

        robot_thata = Chassis.descriptors[GetChassisdescriptorIndex(pose.z())].image_or;
        int index_x = (int)round((pose.x() - occupancyMap2D.info.origin.position.x) / _mapResolution);
        int index_y = (int)round((pose.y() - occupancyMap2D.info.origin.position.y) / _mapResolution);
        imageROI_show = ImageMap_show(Rect(index_y -robot_thata.cols/2,index_x -robot_thata.rows/2 ,robot_thata.cols,robot_thata.rows));
        robot_thata.copyTo(imageROI_show);
        ImageMap_show_ = ImageMap_show_ + ImageMap_show;
    }
     if(pubImage_show.getNumSubscribers() !=0)
          {
             cv_bridge::CvImage out_z_image;
             out_z_image.header.stamp   = ros::Time::now();// Same timestamp and tf frame as input image
             out_z_image.header.frame_id   = "map"; // Same timestamp and tf frame as input image
             out_z_image.encoding = "mono8";
             out_z_image.image    = ImageMap_show_; // Your cv::Mat
             pubImage_show.publish(out_z_image.toImageMsg());
          }

}

bool isInRange(const Vect_3d& current_state, const Vect_3d& goal_state) {
  //int random = rand() % 10 + 1;
//   std::cout<<"current_state.x() - goal_state.x()"<<current_state.x() - goal_state.x()<<std::endl;
//   double dx = std::abs(current_state.x() - goal_state.x()) ;
//   double dy = std::abs(current_state.y() - goal_state.y()) ;

  double dist=  (current_state.head(2) - goal_state.head(2)).norm() ;
//   double dist= (dx * dx) + (dy * dy) ;
   
  double datatheata=std::abs(current_state.z()-goal_state.z());
  if(dist<6.0&&datatheata<3.0)//10ｄｅｇｒｅｅ
  return true;
  return false;
}


bool kinodynamic_Search(const  Vect_3d &start_state, const  Vect_3d &goal_state) {
     Timer search_used_time;

    double neighbor_time = 0.0, compute_h_time = 0.0, compute_g_time = 0.0;

    const Vec_3i start_grid_index = State2Index(start_state);
    const Vec_3i goal_grid_index = State2Index(goal_state);

    auto goal_node_ptr = new StateNode(goal_grid_index);
    goal_node_ptr->state_ = goal_state;
    goal_node_ptr->direction_ = StateNode::NO;
    goal_node_ptr->steering_grade_ = 0;

    auto start_node_ptr = new StateNode(start_grid_index);
    start_node_ptr->state_ = start_state;
    start_node_ptr->steering_grade_ = 0;
    start_node_ptr->direction_ = StateNode::NO;
    start_node_ptr->node_status_ = StateNode::IN_OPENSET;
    start_node_ptr->intermediate_states_.emplace_back(start_state);
    start_node_ptr->g_cost_ = 0.0;
    start_node_ptr->f_cost_ = ComputeH(start_node_ptr, goal_node_ptr);

    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    openset_.clear();
    openset_.insert(std::make_pair(0, start_node_ptr));

    std::vector<StateNode::Ptr> neighbor_nodes_ptr;
    StateNode::Ptr current_node_ptr;
    StateNode::Ptr neighbor_node_ptr;

    int count = 0;
    int num = 0;
    while (!openset_.empty()) {
        
        current_node_ptr = openset_.begin()->second;
    
        current_node_ptr->node_status_ = StateNode::IN_CLOSESET;

        openset_.erase(openset_.begin());
       
       
        if ((current_node_ptr->state_.head(2) - goal_node_ptr->state_.head(2)).norm() <= 5.0&&use_rspath) {
           

            double rs_length = 0.0;
            if (AnalyticExpansions(current_node_ptr, goal_node_ptr, rs_length)) {   // have opencv  
              
                terminal_node_ptr_ = goal_node_ptr;

                StateNode::Ptr grid_node_ptr = terminal_node_ptr_->parent_node_;
                while (grid_node_ptr != nullptr) {
                    grid_node_ptr = grid_node_ptr->parent_node_;
                    path_length_ = path_length_ + segment_length_;
                }
                path_length_ = path_length_ - segment_length_ + rs_length;
                
                std::cout << "ComputeH use time(ms): " << compute_h_time << std::endl;
                std::cout << "check collision use time(ms): " << check_collision_use_time << std::endl;
                std::cout << "GetNeighborNodes use time(ms): " << neighbor_time << std::endl;
                std::cout << "average time of check collision(ms): "
                          << check_collision_use_time / num_check_collision
                          << std::endl;
                std::cout << "num_check_collision(ms): " << num_check_collision << std::endl;
                ROS_INFO("\033[1;32m --> Time in Hybrid A star is %f ms, path length: %f  \033[0m\n",
                         search_used_time.End(), path_length_);

                check_collision_use_time = 0.0;
                num_check_collision = 0.0;
                return true;
            }
        }
        if(!use_rspath)
        {
            if((current_node_ptr->state_.head(2) - goal_node_ptr->state_.head(2)).norm() <= 0.8 && std::abs(current_node_ptr->state_[2]-goal_node_ptr->state_[2])<=0.6)
            {   
                goal_node_ptr->parent_node_ = current_node_ptr;
                terminal_node_ptr_ = goal_node_ptr;
                StateNode::Ptr grid_node_ptr = terminal_node_ptr_->parent_node_;
                while (grid_node_ptr != nullptr) {
                    grid_node_ptr = grid_node_ptr->parent_node_;
                    path_length_ = path_length_ + segment_length_;
                }
                path_length_ = path_length_ - segment_length_;
                return true;

            }
                
        }
        // else{
        //         if(isInRange(current_node_ptr->state_,goal_node_ptr->state_))
        //            return true;
                   

        // }

       
        Timer timer_get_neighbor;
 
        GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr);
 
        neighbor_time = neighbor_time + timer_get_neighbor.End();

        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes_ptr[i];

            Timer timer_compute_g;
            
            const double neighbor_edge_cost =ComputeG(current_node_ptr, neighbor_node_ptr) ;//0 
           
            compute_g_time = compute_g_time + timer_get_neighbor.End();

            Timer timer_compute_h;
            const double current_h = ComputeH(current_node_ptr, goal_node_ptr) * tie_breaker_;
            compute_h_time = compute_h_time + timer_compute_h.End();

            const Vec_3i &index = neighbor_node_ptr->grid_index_;
            num++;

            if (state_node_map_[index.x()][index.y()][index.z()] == nullptr) {
                neighbor_node_ptr->g_cost_ = current_node_ptr->g_cost_ + neighbor_edge_cost;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                neighbor_node_ptr->f_cost_ = neighbor_node_ptr->g_cost_ + current_h;
                openset_.insert(std::make_pair(neighbor_node_ptr->f_cost_, neighbor_node_ptr));
                state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                continue;
               } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_OPENSET) {
                double g_cost_temp = current_node_ptr->g_cost_ + neighbor_edge_cost;

                if (state_node_map_[index.x()][index.y()][index.z()]->g_cost_ > g_cost_temp) {
                    neighbor_node_ptr->g_cost_ = g_cost_temp;
                    neighbor_node_ptr->f_cost_ = g_cost_temp + current_h;
                    neighbor_node_ptr->parent_node_ = current_node_ptr;
                    neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;

                    delete state_node_map_[index.x()][index.y()][index.z()];
                    state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                } else {
                    delete neighbor_node_ptr;
                }
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_CLOSESET) {
                delete neighbor_node_ptr;
                continue;
            }

        }

        count++;
        if (count > 50000) {
            ROS_WARN("Exceeded the number of iterations, the search failed");
            return false;
        }
   
    }

    return false;
}



    void ImageMapCallback (const sensor_msgs::ImageConstPtr& imag)
    {
        std::lock_guard<std::mutex> lock(mtx);
        // localMapFrame_ = image->header.frame_id;
        // ros::Time mapTime = image->header.stamp;
        // std::string map_frame = image->header.frame_id;
        ImageMap.release();
        ImageMap = cv::Mat::zeros(400, 400, CV_8UC1);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imag, sensor_msgs::image_encodings::TYPE_8UC1);
	    ImageMap = cv_ptr -> image;

        // ImageMap = cv_bridge::toCvShare(imag,sensor_msgs::image_encodings::TYPE_8UC1)->image;
        receive_imagemap = true;
    }
    void ImageMapCostCallback (const sensor_msgs::ImageConstPtr& imag)
    {    
         std::lock_guard<std::mutex> lock(mtx);
         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imag, sensor_msgs::image_encodings::TYPE_8UC1);
	     ImageMapCost = cv_ptr -> image;
    }
    void ImagePathMapCallback (const sensor_msgs::ImageConstPtr& imag)
    {     
         std::lock_guard<std::mutex> lock(mtx);
         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imag, sensor_msgs::image_encodings::TYPE_8UC1);
	     ImagePathMap = cv_ptr -> image;
    }

    void ImagePathMapBoundCallback (const sensor_msgs::ImageConstPtr& imag)
    {
         std::lock_guard<std::mutex> lock(mtx);
         ImagePathMapBound.release();
          ImagePathMapBound = cv::Mat::zeros(103, 103, CV_32FC1);
         cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(imag, sensor_msgs::image_encodings::TYPE_32FC1);
	     ImagePathMapBound = cv_ptr1 -> image;
    }

    int GetChassisdescriptorIndex(const float& theta)
    {    
        float angle = Chassis.NormalizeAngle(theta) *180/PI_;
        return Chassis.GetAngleIdxFast(angle);
    }

    



    void pathHandler(const nav_msgs::Path::ConstPtr& pathMsg)
    {
        std::lock_guard<std::mutex> lock(mtx); //

        if (pathMsg->poses.size() <= 1)
        {
            ROS_WARN("Empty global path received.");
            return;
        }

        globalPathMessage = *pathMsg;

        // treate the original centerPath from open_planner as searched path
        openPlannerRollOut.run(transform, globalPathMessage, globalPath, searchedPath, remainingPath);
        //cout<<"remainingPath ："<<remainingPath.poses[0].pose.position.x;
        executePath = combinePaths(searchedPath, remainingPath);
        searchedPath = processPath(searchedPath);
        remainingPath = processPath(remainingPath);
        //cout<<"remainingPath 1："<<remainingPath.poses[0].pose.position.x;

        pubSearchedPath.publish(executePath);

        // subGlobalPath.shutdown();
        ROS_INFO("\033[1;32m Global Path recieved. \033[0m");
    }

    void mapHandler(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
       
        configurationSpace._mapResolution=_mapResolution;
        occupancyMap2D = *mapMsg;

        map_x_lower_ = occupancyMap2D.info.origin.position.x;
        map_x_upper_ = _mapResolution*_local_map_grid_num;
        map_y_lower_ = occupancyMap2D.info.origin.position.y;
        map_y_upper_ =  _mapResolution*_local_map_grid_num;
        STATE_GRID_RESOLUTION_ = _mapResolution;

        configurationSpace.updateGrid(occupancyMap2D);
        // nav_msgs::OccupancyGrid::Ptr map_grig=;
       // configurationSpace.updateGrid(mapMsg);//?????????????????????????????????
    }

        void path_mapHandler(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
       
        //configurationSpace._mapResolution=_mapResolution;
        Path_Map2D = *mapMsg;
        //configurationSpace.updateGrid(occupancyMap2D);
        // nav_msgs::OccupancyGrid::Ptr map_grig=;
       // configurationSpace.updateGrid(mapMsg);//?????????????????????????????????
    }
    
    struct CompareNodes {
    /// Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  // bool operator()(const Node2D* lhs, const Node2D* rhs) const {
  //   return lhs->getC() > rhs->getC();
  // }
  };

    nav_msgs::Path combinePaths(nav_msgs::Path pathFront, nav_msgs::Path pathBack)
    {
        nav_msgs::Path pathOut = pathFront;

        if (pathBack.poses.size() > 0)
            pathOut.poses.insert(pathOut.poses.end(), pathBack.poses.begin(), pathBack.poses.end());

        return processPath(pathOut);
    }

    float distance(state_t* state_from, state_t* state_to){
        return sqrt((state_to->x-state_from->x)*(state_to->x-state_from->x) + 
                    (state_to->y-state_from->y)*(state_to->y-state_from->y) + 
                    (state_to->z-state_from->z)*(state_to->z-state_from->z));
    }

    bool isIncollision(float x, float y)
    {
        // std::cout<<"x,y"<<x<<"yy"<<y<<std::endl;
        int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
        int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);
        //  std::cout<<"occupancyMap2D"<<occupancyMap2D.info.origin.position.x<<std::endl;
        // std::cout<<"index_x"<<index_x<<std::endl;
        // std::cout<<"index_y"<<index_y<<std::endl;
        int index = index_x + index_y * occupancyMap2D.info.width;

        if (index_x < 0 || index_x >= occupancyMap2D.info.width ||
            index_y < 0 || index_y >= occupancyMap2D.info.height)
            return false;
        
        if (occupancyMap2D.data[index] == 100)
            return true;
        else
            return false;

    }

    bool isIncollision_Byimage(const float &x, const float &y ,const float &yaw)
    {   
        Timer timer;

        cv::Mat robot_thata ;
        // cv::Mat ImageMap_ = ImageMap.clone() ;
       
        robot_thata = Chassis.descriptors[GetChassisdescriptorIndex(yaw)].image_collision;
        // std::cout<<"yaw test: "<<(yaw*180/3.1415)<<std::endl;

        int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
        int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);

        if (index_x < ceil(robot_thata.cols/2) || index_x >= occupancyMap2D.info.width -floor(robot_thata.cols/2) ||
            index_y < ceil(robot_thata.rows/2) || index_y >= occupancyMap2D.info.height- floor(robot_thata.rows/2))
            return false;

        cv::Mat imageROI = ImageMap(Rect(index_y -robot_thata.cols/2,index_x -robot_thata.rows/2 ,robot_thata.cols,robot_thata.rows));

        cv::Mat result_mat = imageROI.mul(robot_thata);
        double max_val = 0;
        cv::minMaxLoc(result_mat,NULL,&max_val,NULL,NULL);
        
        check_collision_use_time += timer.End();
        num_check_collision++;
       //  cout<<"ImagePathMapBound_____________"<<ImageMap.at<int>(10,10)<<endl;

        if (max_val > 200) //0 200 255
            return true;
        else
            return false;
                
        // std::cout<<"max_val test: "<<max_val<<std::endl;
        // robot_thata.copyTo(imageROI);
        // ImageMapshow = ImageMapshow + ImageMap_;
        // if(pubImage_robot.getNumSubscribers() !=0)
        //   {
        //      cv_bridge::CvImage out_z_image;
        //      out_z_image.header.stamp   = ros::Time::now();// Same timestamp and tf frame as input image
        //      out_z_image.header.frame_id   = "map"; // Same timestamp and tf frame as input image
        //      out_z_image.encoding = "mono8";
        //      out_z_image.image    = ImageMapshow; // Your cv::Mat
        //      pubImage_robot.publish(out_z_image.toImageMsg());
        //   }


    }

    bool findlowobpose_Byimage(const float &x, const float &y ,const float &yaw)
    {   

        cv::Mat robot_thata ;
        // cv::Mat ImageMap_ = ImageMap.clone() ;
       
        robot_thata = Chassis.descriptors[GetChassisdescriptorIndex(yaw)].image_collision;
        // std::cout<<"yaw test: "<<(yaw*180/3.1415)<<std::endl;

        int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
        int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);

        if (index_x < ceil(robot_thata.cols/2) || index_x >= occupancyMap2D.info.width -floor(robot_thata.cols/2) ||
            index_y < ceil(robot_thata.rows/2) || index_y >= occupancyMap2D.info.height- floor(robot_thata.rows/2))
            return false;

        cv::Mat imageROI = ImageMap(Rect(index_y -robot_thata.cols/2,index_x -robot_thata.rows/2 ,robot_thata.cols,robot_thata.rows));

        double val = imageROI.dot(robot_thata);
        
       //  cout<<"ImagePathMapBound_____________"<<ImageMap.at<int>(10,10)<<endl;

        if (val > 0) //0 200 255
            return true;
        else
            return false;
                

    }

 bool isBeyondBoundary_Byimage(const Vec_2d &pt) const {
                int index_x = (pt.x() - occupancyMap2D.info.origin.position.x) / _mapResolution;
                int index_y = (pt.y() - occupancyMap2D.info.origin.position.y) / _mapResolution;
             if (index_x < 0 || index_y < 0 || index_x >= _local_map_grid_num || index_y >= _local_map_grid_num)
                return true;
                return false;

}

        bool opopopopisBeyondBoundary_Byimage(const float &x, const float &y ,const float &yaw)
    {   

        return false;

        cv::Mat robot_thata ;
         cv::Mat ImageMap_ = ImagePathMapBound.clone() ;
       
        robot_thata = Chassis.descriptors[GetChassisdescriptorIndex(yaw)].image_or;
       // cout<<"Image robot0 "<<robot_thata.at<int>(10,10)<<endl;
        // robot_thata.convertTo(robot_thata, CV_32FC1, 0.0);
       //  cout<<"Image robot1 "<<robot_thata.at<float>(10,10)<<endl;
        // std::cout<<"yaw test: "<<(yaw*180/3.1415)<<std::endl;

        int index_x = (int)round((x - Path_Map2D.info.origin.position.x) / Path_Map2D.info.resolution);
        int index_y = (int)round((y - Path_Map2D.info.origin.position.y) / Path_Map2D.info.resolution);

        if (index_x < ceil(robot_thata.cols/2) || index_x >= occupancyMap2D.info.width -floor(robot_thata.cols/2) ||
            index_y < ceil(robot_thata.rows/2) || index_y >= occupancyMap2D.info.height- floor(robot_thata.rows/2))
            return false;

        cv::Mat imageROI = ImagePathMapBound(Rect(index_y -robot_thata.cols/2,index_x -robot_thata.rows/2 ,robot_thata.cols,robot_thata.rows));
      //  cout<<"ImagePathMapBound row "<<ImagePathMapBound.rows<<endl;
       // cout<<"ImageMap row "<<ImageMap.rows<<endl;
       //  cout<<"ImagePathMapBound  "<<ImagePathMapBound.at<float>(10,10)<<endl;
        //  cout<<"ImagePathMapBound int "<<ImagePathMapBound.at<int>(10,10)<<endl;
         

       //   cout<<"ImagePathMapBound.type() "<<ImagePathMapBound.type()<<endl;
           //cout<<"robot_thata.type() "<<robot_thata.type()<<endl;
        
         cv::Mat imageROI_ = ImageMap_(Rect(index_y -robot_thata.cols/2,index_x -robot_thata.rows/2 ,robot_thata.cols,robot_thata.rows));
         int result = imageROI.dot(robot_thata);


          std::cout<<"result test: "<<result<<std::endl;
          //robot_thata.copyTo(imageROI_);
        
        if(pubImage_robot.getNumSubscribers() !=0)
          {
             cv_bridge::CvImage out_z_image;
             out_z_image.header.stamp   = ros::Time::now();// Same timestamp and tf frame as input image
             out_z_image.header.frame_id   = "map"; // Same timestamp and tf frame as input image
             out_z_image.encoding =  sensor_msgs::image_encodings::TYPE_32FC1; 
             out_z_image.image    = imageROI_; // Your cv::Mat
             pubImage_robot.publish(out_z_image.toImageMsg());
          }


        if (result > 0) //
            return true;
        else
            return false;


    }

     bool isIncollision1(float x, float y)
    {
        int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
        int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);
        int index = index_x + index_y * occupancyMap2D.info.width;

        if (index_x < 0 || index_x >= occupancyMap2D.info.width ||
            index_y < 0 || index_y >= occupancyMap2D.info.height)
            return false;

        //if (occupancyMap2D.data[index] == 100)
          if (occupancyMap2D.data[index] >0)
            return true;
        else
            return false;
    }

    float getCloseCollisionCost(float x, float y)
    {
        int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
        int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);
        int index = index_x + index_y * occupancyMap2D.info.width;

        if (index_x < 0 || index_x >= occupancyMap2D.info.width ||
            index_y < 0 || index_y >= occupancyMap2D.info.height)
            return false;

        if (occupancyMap2D.data[index] > 0)
            return float(occupancyMap2D.data[index]);
        else
            return 0;
    }

    bool isCloseCollision(float x, float y)
    {
        int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
        int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);
        int index = index_x + index_y * occupancyMap2D.info.width;

        if (index_x < 0 || index_x >= occupancyMap2D.info.width ||
            index_y < 0 || index_y >= occupancyMap2D.info.height)
            return false;

        if (occupancyMap2D.data[index] > 50)
            return true;
        else
            return false;
    }


      void  reset() 
      {
         // Reset kdtree first.
          if (kd_tree_) kd_free(kd_tree_);
           kd_tree_ = kd_create(2);
      }

    void buildAdjacencyMatrix()
    {
        kd_tree_ = NULL;
        reset();
        openPlannerRollOut.run(transform, globalPathMessage, globalPath, centerPath, remainingPath, alternativePaths);

        // clear memory
        for (int i = 0; i < nodeList.size(); ++i)
        {
            state_t *stateCur = nodeList[i];
            delete stateCur;
        }
        nodeList.clear();

        // allocate vector size
        adjacency_width_grid = alternativePaths.size();
        adjacency_length_grid = alternativePaths[0].poses.size();

        adjacencyMatrix.resize(adjacency_width_grid);
        for (int i = 0; i < adjacency_width_grid; ++i)
            adjacencyMatrix[i].resize(adjacency_length_grid);


        // create new states for adjacency matrix
        for (int i = 0; i < alternativePaths.size(); ++i)
        {
            for (int j = 0; j < alternativePaths[i].poses.size(); ++j)
            {
                state_t *newState = new state_t;
                newState->x = alternativePaths[i].poses[j].pose.position.x;
                newState->y = alternativePaths[i].poses[j].pose.position.y;
                newState->z = 0;
                newState->theta = tf::getYaw(alternativePaths[i].poses[j].pose.orientation);//弧度

                newState->idx = i;
                newState->idy = j;
                newState->validFlag = true;
                newState->stateId = nodeList.size();

                nodeList.push_back(newState);
                adjacencyMatrix[i][j] = newState;

                kd_insert3(kd_tree_,newState->x,newState->y, newState->z, newState);
            }
        }
    }

    void connectAdjacencyMatrix()
    {
        // check node collision first
        for (int i = 0; i < nodeList.size(); ++i)
        {
            if (isIncollision(nodeList[i]->x, nodeList[i]->y))
            {    
                PointType *newpoint = new PointType;
                // nodeList[i]->idx;
                // nodeList[j]->idy;
                newpoint->x = adjacencyMatrix[0][nodeList[i]->idy]->x;
                newpoint->y = adjacencyMatrix[0][nodeList[i]->idy]->y;
                obtest.push_back(newpoint);
                

                nodeList[i]->validFlag = false;
                continue;
            }
        }

        int connection_length = 1;
        // connect adjacency matrix
        for (int i = 0; i < adjacency_width_grid; ++i)
        {
            for (int j = 0; j < adjacency_length_grid; ++j)
            {
                if (adjacencyMatrix[i][j]->validFlag == false)
                    continue;

                state_t* state_from = adjacencyMatrix[i][j];

                for (int m = -connection_length; m <= connection_length; ++m)
                {
                    for (int n = -connection_length; n <= connection_length; ++n)
                    {
                        if ((m == 0 && n == 0) ) // do not add itself  || abs(m)==abs(n)
                            continue;

                        int id_x = i + m;
                        int id_y = j + n;
                        if (id_x < 0 || id_x >= adjacency_width_grid || id_y < 0 || id_y >= adjacency_length_grid)
                            continue;

                        if (adjacencyMatrix[id_x][id_y]->validFlag == false)
                            continue;

                        state_t* state_to = adjacencyMatrix[id_x][id_y];

                        float edgeCosts[NUM_COSTS];
                        if(edgePropagation(state_from, state_to, edgeCosts) == true)///////在搜索阶段加上
                        {
                            neighbor_t thisNeighbor;
                            for (int q = 0; q < NUM_COSTS; ++q)
                                thisNeighbor.edgeCosts[q] = edgeCosts[q];
                            thisNeighbor.neighbor = state_to;
                            state_from->neighborList.push_back(thisNeighbor);
                        }
                    }
                }
            }
        } 
 //       std::cout<<"adjacencyMatrix="<<adjacencyMatrix[8].size()<<std::endl;
    }


    bool edgePropagation(state_t *state_from, state_t *state_to, float edgeCosts[NUM_COSTS]){
        // 0. initialize edgeCosts
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = 0;
        // 1. segment the edge for collision checking
        int steps = round(distance(state_from, state_to) / _mapResolution);
        float stepX = (state_to->x - state_from->x) / (float)steps;
        float stepY = (state_to->y - state_from->y) / (float)steps;
        float stepZ = (state_to->z - state_from->z) / (float)steps;
        // 2. allocate memory for a state, this state must be deleted after collision checking
        state_t *stateCur = new state_t;;
        stateCur->x = state_from->x;
        stateCur->y = state_from->y;
        stateCur->z = state_from->z;

        // 3. collision checking loop
        for (int stepCount = 0; stepCount < steps; ++stepCount)
        {
            stateCur->x += stepX;
            stateCur->y += stepY;
            stateCur->z += stepZ;

            if (isIncollision(stateCur->x, stateCur->y))
            {
                delete stateCur;
                return false;
            }

            // close to obstacle cost
            edgeCosts[0] += _mapResolution * getCloseCollisionCost(stateCur->x, stateCur->y);
        }

        // lane change cost
        edgeCosts[1] = (float)abs(state_from->idx - state_to->idx);

        // not center lane cost
        // if (state_from->idx != _rollOutCenter || state_to->idx != _rollOutCenter)
        //     edgeCosts[2] += distance(state_from, state_to);

        // distance cost
        edgeCosts[2] = distance(state_from, state_to); // distance cost

        // treat first column nodes all the same
        if (state_from->idy == 0 && state_to->idy == 0)
            for (int i = 0; i < NUM_COSTS; ++i)
                edgeCosts[i] = 0;

        // treat last column nodes all the same
        if (state_from->idy == adjacency_length_grid - 1 && state_to->idy == adjacency_length_grid - 1)
            for (int i = 0; i < NUM_COSTS; ++i)
                edgeCosts[i] = 0;

        delete stateCur;
        return true;
    }

     bool getNearestState(const state_t* state,
                                        state_t** s_res) {
         // it seems kdtree lib  can not deal with empty tree, put a guard check here.
      
         kdres* nearest = kd_nearest3(kd_tree_, state->x, state->y, state->z);
         if (kd_res_size(nearest) <= 0) {
         kd_res_free(nearest);
         return false;
         }
         *s_res = (state_t*)kd_res_item_data(nearest);
         kd_res_free(nearest);
         return true;
       }
   

   bool isongraph(Node3D* nSucc)
   {

           state_t *Succ=new state_t;
        Succ->x=nSucc->getX();
         Succ->y=nSucc->getY();
          Succ->z=0;
        state_t  *NearestState=NULL;
        getNearestState(Succ,&NearestState);
        if(distance(Succ,NearestState)<0.4)
        return true;
        else return false;

   }
   bool  isnodeequation(Node3D  a,Node3D  b)
   {
       if(a.getX()==b.getX()&&a.getY()==b.getY()&&a.getT()==b.getT())
       return true;
       return false;

   }
Node3D* hybridAStar(Node3D& start,
                    const Node3D& goal,
                    Node3D* nodes3D,
                    int width,
                    int height,
                    CollisionDetection& configurationSpace,
                    Visualize& visualization_) {   
        //   std::cout<<"hybrid_star"<<std::endl;

  // // PREDECESSOR AND SUCCESSOR INDEX
   int iPred, iSucc;
   float newG;
  // // Number of possible directions, 3 for forward driving and an additional 3 for reversing
   int dir = Constants::reverse ? 6 : 3;
  // // Number of iterations the algorithm has run for stopping based on Constants::iterations
   int iterations = 0;

  // // VISUALIZATION DELAY
//    ros::Duration d(0.003);

  // // OPEN LIST AS BOOST IMPLEMENTATION
   typedef boost::heap::binomial_heap<Node3D*,
           boost::heap::compare<CompareNodes>
          >  priorityQueue;
      priorityQueue O;

  // // update h value
  // updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);
  // // mark start as open
   start.open();
  // // push on priority queue aka open list
   O.push(&start);
   iPred = start.setIdx(width, height, occupancyMap2D.info.origin.position.x, occupancyMap2D.info.origin.position.y);
   nodes3D[iPred] = start;

  // // NODE POINTER
   Node3D* nPred;
   Node3D* nSucc;

  // // float max = 0.f;

  // // continue until O empty
   while (!O.empty()) {
  //   //    // DEBUG
           //  Node3D* pre = nullptr;
  //   //    Node3D* succ = nullptr;

  //   //    std::cout << "\t--->>>" << std::endl;

  //   //    for (priorityQueue::ordered_iterator it = O.ordered_begin(); it != O.ordered_end(); ++it) {
  //   //      succ = (*it);
  //   //      std::cout << "VAL"
  //   //                << " | C:" << succ->getC()
  //   //                << " | x:" << succ->getX()
  //   //                << " | y:" << succ->getY()
  //   //                << " | t:" << helper::toDeg(succ->getT())
  //   //                << " | i:" << succ->getIdx()
  //   //                << " | O:" << succ->isOpen()
  //   //                << " | pred:" << succ->getPred()
  //   //                << std::endl;

  //   //      if (pre != nullptr) {

  //   //        if (pre->getC() > succ->getC()) {
  //   //          std::cout << "PRE"
  //   //                    << " | C:" << pre->getC()
  //   //                    << " | x:" << pre->getX()
  //   //                    << " | y:" << pre->getY()
  //   //                    << " | t:" << helper::toDeg(pre->getT())
  //   //                    << " | i:" << pre->getIdx()
  //   //                    << " | O:" << pre->isOpen()
  //   //                    << " | pred:" << pre->getPred()
  //   //                    << std::endl;
  //   //          std::cout << "SCC"
  //   //                    << " | C:" << succ->getC()
  //   //                    << " | x:" << succ->getX()
  //   //                    << " | y:" << succ->getY()
  //   //                    << " | t:" << helper::toDeg(succ->getT())
  //   //                    << " | i:" << succ->getIdx()
  //   //                    << " | O:" << succ->isOpen()
  //   //                    << " | pred:" << succ->getPred()
  //   //                    << std::endl;

  //   //          if (pre->getC() - succ->getC() > max) {
  //   //            max = pre->getC() - succ->getC();
  //   //          }
  //   //        }
  //   //      }

  //   //      pre = succ;
  //   //    }

  //   // pop node with lowest cost from priority queue
      nPred = O.top();
  //   // set index
     iPred = nPred->setIdx(width, height, occupancyMap2D.info.origin.position.x, occupancyMap2D.info.origin.position.y);
     iterations++;

  //   // RViz visualization
    if (Constants::visualization) {
      visualization_.publishNode3DPoses(*nPred);
    //  visualization_.publishNode3DPose(*nPred);
    //   d.sleep();
    }

  //   // _____________________________
  //   // LAZY DELETION of rewired node
  //   // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
  //   // _________________
  //   // EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen()) {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST

      // isnodeequation(*nPred,goal) 
      std::cout<<"not iterations:"<<iterations<<std::endl;
      if (nPred->isInRange(goal)  || iterations > Constants::iterations) {
        // DEBUG
        std::cout<<"iterations:"<<iterations<<std::endl;
        visualization_.publishNode3DPose(*nPred);
        return nPred;
      }

        //     // ____________________
       //     // CONTINUE WITH SEARCH
      else {
        // _______________________
        // SEARCH WITH DUBINS SHOT
        std::cout<<"not iterations222:"<<iterations<<std::endl;
        if ( false) {  /// nSucc->isInRange(goal)

              std::cout<<"iterations22:::"<<iterations<<std::endl;
          //nSucc = dubinsShot(*nPred, goal, configurationSpace);

           //if (nSucc != nullptr && *nSucc == goal) {
            //DEBUG
             // std::cout << "max diff " << max << std::endl;
             visualization_.publishNode3DPose(*nSucc);
               return nSucc;
         
        }

         //       // ______________________________
         //       // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);//************************************************
          // set index of the successor
          iSucc = nSucc->setIdx(width, height, occupancyMap2D.info.origin.position.x, occupancyMap2D.info.origin.position.y);

          // ensure successor is on grid and traversable
            
            // isongraph(nSucc);
        //原始碰撞检测
        //    if (nSucc->isOnGrid(width, height,occupancyMap2D.info.origin.position.x, occupancyMap2D.info.origin.position.y) && 
        //                            configurationSpace.isTraversable(nSucc)&&isongraph(nSucc)) {

        if (nSucc->isOnGrid(width, height,occupancyMap2D.info.origin.position.x, occupancyMap2D.info.origin.position.y) && 
                                   !isIncollision_Byimage(nSucc->getX(), nSucc->getY(), nSucc->getT())&&isongraph(nSucc)) {

          //    if (nSucc->isOnGrid(width, height,occupancyMap2D.info.origin.position.x, occupancyMap2D.info.origin.position.y) ) {//********
            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

              // calculate new G value
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

                // calculate H value
                //updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);

                // if the successor is in the same cell but the C value is larger
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  delete nSucc;
                  continue;
                }
              //               // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                  nSucc->setPred(nPred->getPred());
                }

                if (nSucc->getPred() == nSucc) {
                  std::cout << "looping";
                }

                // put successor on open list
                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                O.push(&nodes3D[iSucc]);
                delete nSucc;
              } else { delete nSucc; }
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) {
    return nullptr;
  }

  return nullptr;
}



//  void tracePath(const Node3D* node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>());
void tracePath(const Node3D* node, int i = 0 , std::vector<Node3D> path_node= std::vector<Node3D>()) {
  if (node == nullptr) {
      std::cout<<"node is  empty"<<std::endl;
    this->path_node = path_node;
    return;
  }

  i++;
  path_node.push_back(*node);
  tracePath(node->getPred(), i, path_node);
}


     void plan()
   {
      int width =  occupancyMap2D.info.width*_mapResolution;
      int height = occupancyMap2D.info.height*_mapResolution;
      int depth = Constants::headings;
      int length = width * height * depth;
      // define list pointers and initialize lists
      Node3D* nodes3D = new Node3D[length]();
      //Node2D* nodes2D = new Node2D[width * height]();
      state_t *startState = adjacencyMatrix[int(adjacency_width_grid/2)][0];
      state_t *goalState = adjacencyMatrix[0][adjacency_length_grid-2];
      float x = goalState->x / Constants::cellSize;
      float y = goalState->y / Constants::cellSize;
     float t = goalState->theta;
      // std::cout<<"oalState->theta;="<<t<<std::endl;
      //   // set theta to a value (0,2PI]

       t = Helper::normalizeHeadingRad(t);

      //  std::cout<<"oalState-a;="<<t<<std::endl;
     const Node3D nGoal(x, y, t, 0, 0, nullptr);
     Node3D vi=nGoal;
     visualization_.publishNode3DPose_st(vi);

      //    // __________
      //     // DEBUG GOAL
      //    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);
     

    //  // _________________________
    //  // retrieving start position   
    // float x_ =startState->x / Constants::cellSize;
    // float y_ = startState->y / Constants::cellSize;
    // float t_ = startState->theta;

    float x_ = robotPoint.x / Constants::cellSize;
    float y_ = robotPoint.y / Constants::cellSize;
    float t_ = robotPoint.intensity;


    //  // set theta to a value (0,2PI]
      t_ = Helper::normalizeHeadingRad(t_);
      Node3D nStart(x_, y_, t_, 0, 0, nullptr);
    //   // ___________
    //  // DEBUG START
    //  //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    //  // ___________________________
    //  // START AND TIME THE PLANNING
      ros::Time t0 = ros::Time::now();

    //  // CLEAR THE VISUALIZATION
      visualization_.clear();
 
    //  // CLEAR THE PATH
      path.clear();
    //  smoothedPath.clear();
    //  // FIND THE PATH
    Node3D* nSolution = hybridAStar(nStart, nGoal, nodes3D, width, height, configurationSpace,visualization_);
    //  // TRACE THE PATH
        tracePath(nSolution);
    //  // CREATE THE UPDATED PATH
       path.updatePath_(getPath());
    //  // SMOOTH THE PATH
    //  smoother.smoothPath(voronoiDiagram);
    //  // CREATE THE UPDATED PATH
    //  smoothedPath.updatePath(smoother.getPath());
    //  ros::Time t1 = ros::Time::now();
    //  ros::Duration d(t1 - t0);
    //  std::cout << "TIME in ms: " << d * 1000 << std::endl;

    //  // _________________________________
    //  // PUBLISH THE RESULTS OF THE SEARCH
      path.publishPath();
     path.publishPathNodes();
       path.publishPathVehicles();
    //  smoothedPath.publishPath();
    // smoothedPath.publishPathNodes();
    // smoothedPath.publishPathVehicles();
    // visualization.publishNode3DCosts(nodes3D, width, height, depth);
    // visualization.publishNode2DCosts(nodes2D, width, height);



     delete [] nodes3D;
     //delete [] nodes2D;
   
     //}

   }

    bool searchAdjacencyMatrix()
    {   
        // 1. reset costs
        for (int i = 0; i < nodeList.size(); ++i)
            for (int j = 0; j < NUM_COSTS; ++j)
                nodeList[i]->costsToRoot[j] = FLT_MAX;

        // 2. define start state
        state_t *startState = adjacencyMatrix[int(adjacency_width_grid/2)][0];
        state_t *startState12=new state_t;
        startState12->x=0.1;
        startState12->y=0.1;
        startState12->z=0.1;
        state_t  *State_=NULL;
        getNearestState(startState12,&State_);
        //    std::cout<<"State_"<<State_->x<<std::endl;
        //    std::cout<<"State_idx"<<State_->idx<<std::endl;

        for (int i = 0; i < NUM_COSTS; ++i)
            startState->costsToRoot[i] = 0;

        // 3. search graph
        vector<state_t*> Queue;
        Queue.push_back(startState);

        while(Queue.size() > 0 && ros::ok())
        {       
            // find the state that can offer lowest cost in this depth and remove it from Queue
            state_t *fromState = minCostStateInQueue(Queue);
            Queue.erase(remove(Queue.begin(), Queue.end(), fromState), Queue.end());
            // loop through all neighbors of this state
            for (int i = 0; i < fromState->neighborList.size(); ++i)
            {   
                state_t *toState = fromState->neighborList[i].neighbor;
                
                if (toState->validFlag == false)
                    continue;
                // Loop through cost hierarchy
                for (int costIndex = 0; costIndex < NUM_COSTS; ++costIndex)
                {   
                    float thisCost = fromState->costsToRoot[costIndex] + fromState->neighborList[i].edgeCosts[costIndex];
                    // If cost can be improved, update this node with new costs
                    if (thisCost < toState->costsToRoot[costIndex]){
                        updateCosts(fromState, toState, i); // update toState's costToRoot
                        toState->parentState = fromState; // change parent for toState
                        Queue.push_back(toState);
                    }
                     // If cost is same, go to compare secondary cost
                     else if (thisCost == toState->costsToRoot[costIndex]){
                        continue;
                    }
                    // If cost becomes higher, abort this propagation
                    else
                        break;
                }
            }
        }

        // 4. find goal state
        Queue.clear();
        for (int i = 0; i < adjacency_width_grid; ++i)
            Queue.push_back(adjacencyMatrix[i][adjacency_length_grid-1]);

       // state_t* goalState = minCostStateInQueue(Queue);  
        state_t *goalState = adjacencyMatrix[0][adjacency_length_grid-1];

        // 5. extract path
        if (goalState->parentState == NULL)
        {
            failureOccurred();
        } else {
            searchedPath = extractPath(goalState);
            pubSearchedPath.publish(searchedPath);
            executePath = combinePaths(searchedPath, remainingPath);
        }

        return true;
    }

    void failureOccurred()
    {
        if (pointDistance(fixedPoint, robotPoint) > 0.25)
            fixedPoint = robotPoint;
        // search failed, let the robot stay in its position
        nav_msgs::Path pathOut;
        pathOut.header.frame_id = "map";
        pathOut.header.stamp = ros::Time();

        geometry_msgs::PoseStamped poseCur;
        poseCur.header.stamp = ros::Time();
        poseCur.header.frame_id = "map";

        poseCur.pose.position.x = fixedPoint.x;
        poseCur.pose.position.y = fixedPoint.y;
        poseCur.pose.position.z = fixedPoint.z;
        poseCur.pose.orientation = tf::createQuaternionMsgFromYaw(double(fixedPoint.intensity));

        pathOut.poses.push_back(poseCur);

        searchedPath = pathOut;
        executePath = pathOut;
        ROS_WARN("No path found, stay stationary!");
    }

    nav_msgs::Path extractPath(state_t* stateCur)
    {
        nav_msgs::Path pathOut;
        pathOut.header.frame_id = "map";
        pathOut.header.stamp = ros::Time();

        while (ros::ok())
        {
            geometry_msgs::PoseStamped poseCur;
            poseCur.header.stamp = ros::Time();
            poseCur.header.frame_id = "map";

            poseCur.pose.position.x = stateCur->x;
            poseCur.pose.position.y = stateCur->y;
            poseCur.pose.position.z = stateCur->z;

            pathOut.poses.insert(pathOut.poses.begin(), poseCur);

            if (stateCur->parentState == NULL)
                break;
            else
                stateCur = stateCur->parentState;
        }

        pathOut = processPath(pathOut);
        return pathOut;
    }

    void updateCosts(state_t* fromState, state_t* toState, int neighborInd){
        for (int i = 0; i < NUM_COSTS; ++i)
            toState->costsToRoot[i] = fromState->costsToRoot[i] + fromState->neighborList[neighborInd].edgeCosts[i];
    }

    state_t* minCostStateInQueue(vector<state_t*> Queue)
    {
        // Loop through cost hierarchy
        for (int costIndex = 0; costIndex < NUM_COSTS; ++costIndex)
        {
            vector<state_t*> tempQueue;
            float minCost = FLT_MAX;
            // loop through nodes saved in Queue
            for (vector<state_t*>::const_iterator iter2 = Queue.begin(); iter2 != Queue.end(); ++iter2){
                state_t* thisState = *iter2;
                // if cost is lower, we put it in tempQueue in case other nodes can offer same cost
                if (thisState->costsToRoot[costIndex] < minCost){
                    minCost = thisState->costsToRoot[costIndex];
                    tempQueue.clear();
                    tempQueue.push_back(thisState);
                }
                // same cost can be offered by other nodes, we save them to tempQueue for next loop (secondary cost)
                else if (thisState->costsToRoot[costIndex] == minCost)
                    tempQueue.push_back(thisState);
            }
            // Queue is used again for next loop
            Queue.clear();
            Queue = tempQueue;
        }
        // If cost hierarchy is selected correctly, there will be only one element left in Queue (no other ties)
        return Queue[0];
    }

    void visualization()
    {
        if (pubPRMGraph.getNumSubscribers() != 0)
        {
            visualization_msgs::MarkerArray markerArray;
            geometry_msgs::Point p;

            // PRM nodes visualization
            visualization_msgs::Marker markerNode;
            markerNode.header.frame_id = "map";
            markerNode.header.stamp = ros::Time::now();
            markerNode.action = visualization_msgs::Marker::ADD;
            markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
            markerNode.ns = "nodes";
            markerNode.id = 2;
            markerNode.scale.x = 0.03; markerNode.scale.y = 0.03; markerNode.scale.z = 0.03; 
            markerNode.color.r = 0; markerNode.color.g = 1; markerNode.color.b = 1;
            markerNode.color.a = 1;

            for (int i = 0; i < adjacency_width_grid; ++i)
            {
                for (int j = 0; j < adjacency_length_grid; ++j)
                {
                    if (adjacencyMatrix[i][j]->validFlag == false)
                        continue;
                    p.x = adjacencyMatrix[i][j]->x;
                    p.y = adjacencyMatrix[i][j]->y;
                    p.z = adjacencyMatrix[i][j]->z + 0.015;
                    markerNode.points.push_back(p);
                }
            }
          ///
           

            // PRM nodes visualization
            visualization_msgs::Marker markerNode1;
            markerNode1.header.frame_id = "map";
            markerNode1.header.stamp = ros::Time::now();
            markerNode1.action = visualization_msgs::Marker::ADD;
            markerNode1.type = visualization_msgs::Marker::SPHERE_LIST;
            markerNode1.ns = "nodes";
            markerNode1.id = 6;
            markerNode1.scale.x = 0.09; markerNode1.scale.y = 0.09; markerNode1.scale.z = 0.09; 
            markerNode1.color.r = 0; markerNode1.color.g = 1; markerNode1.color.b = 1;
            markerNode1.color.a = 1;

            
                for (int j = 0; j < obtest.size(); ++j)
                {
                   
                    p.x = obtest[j]->x;
                    p.y = obtest[j]->y;
                    p.z = obtest[j]->z + 0.015;
                    markerNode1.points.push_back(p);
                }
            

          ///
            // PRM edge visualization
            visualization_msgs::Marker markerEdge;
            markerEdge.header.frame_id = "map";
            markerEdge.header.stamp = ros::Time::now();
            markerEdge.action = visualization_msgs::Marker::ADD;
            markerEdge.type = visualization_msgs::Marker::LINE_LIST;
            markerEdge.ns = "edges";
            markerEdge.id = 3;
            markerEdge.scale.x = 0.01; markerEdge.scale.y = 0.01; markerEdge.scale.z = 0.01;
            markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
            markerEdge.color.a = 1;

            for (int i = 0; i < adjacency_width_grid; ++i)
            {
                for (int j = 0; j < adjacency_length_grid; ++j)
                {
                    if (adjacencyMatrix[i][j]->validFlag == false)
                        continue;
                    int numNeighbors = adjacencyMatrix[i][j]->neighborList.size();
                    for (int k = 0; k < numNeighbors; ++k)
                    {
                        if (adjacencyMatrix[i][j]->neighborList[k].neighbor->validFlag == false)
                            continue;
                        p.x = adjacencyMatrix[i][j]->x;
                        p.y = adjacencyMatrix[i][j]->y;
                        p.z = adjacencyMatrix[i][j]->z + 0.005;
                        markerEdge.points.push_back(p);
                        p.x = adjacencyMatrix[i][j]->neighborList[k].neighbor->x;
                        p.y = adjacencyMatrix[i][j]->neighborList[k].neighbor->y;
                        p.z = adjacencyMatrix[i][j]->neighborList[k].neighbor->z + 0.005;
                        markerEdge.points.push_back(p);
                    }
                }
            }

            // push to markerarray and publish
            markerArray.markers.push_back(markerNode);
            markerArray.markers.push_back(markerNode1);
            markerArray.markers.push_back(markerEdge);
            pubPRMGraph.publish(markerArray);
        }

        // 4. Single Source Shortest Paths
        if (pubSingleSourcePaths.getNumSubscribers() != 0){

            visualization_msgs::MarkerArray markerArray;
            geometry_msgs::Point p;

            // single source path visualization
            visualization_msgs::Marker markersPath;
            markersPath.header.frame_id = "map";
            markersPath.header.stamp = ros::Time::now();
            markersPath.action = visualization_msgs::Marker::ADD;
            markersPath.type = visualization_msgs::Marker::LINE_LIST;
            markersPath.ns = "path";
            markersPath.id = 4;
            markersPath.scale.x = 0.02; markersPath.scale.y = 0.02; markersPath.scale.z = 0.02;
            markersPath.color.r = 0.3; markersPath.color.g = 0; markersPath.color.b = 1.0;
            markersPath.color.a = 1.0;

            for (int i = 0; i < nodeList.size(); ++i){
                if (nodeList[i]->parentState == NULL)
                    continue;
                if (nodeList[i]->validFlag == false)
                    continue;
                p.x = nodeList[i]->x;
                p.y = nodeList[i]->y;
                p.z = nodeList[i]->z + 0.1;
                markersPath.points.push_back(p);
                p.x = nodeList[i]->parentState->x;
                p.y = nodeList[i]->parentState->y;
                p.z = nodeList[i]->parentState->z + 0.01;
                markersPath.points.push_back(p);
            }
            // push to markerarray and publish
            markerArray.markers.push_back(markersPath);
            pubSingleSourcePaths.publish(markerArray);

            std::cout<<" test: "<<std::endl;
            
        }

                    //转换成图像 1.图像的坐标  2.robot的坐标在图像中间 3.图像的分辨率 
        //     if (pubImage_robot.getNumSubscribers() !=0&&receive_imagemap)
        //   {
        //     cv::Mat robot_thata;


        //      std::cout<<"index test: "<<GetChassisdescriptorIndex(robotPoint.intensity)<<std::endl;

        //     robot_thata = Chassis.descriptors[GetChassisdescriptorIndex(robotPoint.intensity)].image_collision;//.. 
        //      cv_bridge::CvImage out_z_image;
        //      out_z_image.header.stamp   = ros::Time::now();// Same timestamp and tf frame as input image
        //      out_z_image.header.frame_id   = "map"; // Same timestamp and tf frame as input image
        //     out_z_image.encoding = "mono8";
        //      out_z_image.image    = robot_thata; // Your cv::Mat

        //       pubImage_robot.publish(out_z_image.toImageMsg());

        //     int row = ImageMap.rows;
        //    int col = ImageMap.cols;

        //     cv::Mat imageROI = ImageMap(Rect((col-robot_thata.cols)/2,(row-robot_thata.rows)/2,robot_thata.cols,robot_thata.rows));

        //   std::cout<<"imageROI.rows: "<<imageROI.rows<<"imageROI.cols: "<<imageROI.cols<<" index"<<ImageMap.at<int>(100, 100)<<std::endl;
        //   std::cout<<"imageROI.rows: "<<robot_thata.rows<<"imageROI.cols: "<<robot_thata.cols<<" index"<<robot_thata.at< int>(10, 10)<<std::endl;
        // std::cout<<"imageROI.type: "<<imageROI.type()<<std::endl;
        //         std::cout<<"imageROI.type: "<<robot_thata.type()<<std::endl;


     
        //    int test = robot_thata.dot(imageROI);
       
        //    receive_imagemap  = false;
        //   }

    }

    bool getRobotPosition()
    {
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return false; }
        
        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = 0;//transform.getOrigin().z();

        double roll, pitch, yaw;
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(roll, pitch, yaw);
        robotPoint.intensity = NormalizeAngle_pi(yaw);

        if (globalPathMessage.poses.size() == 0)
            return false;

        if (occupancyMap2D.data.size() == 0)
            return false;

        lastRobotPoint = robotPoint;

        return true;
    }

    void publishPath(const ros::TimerEvent& event)
    {   
        std::lock_guard<std::mutex> lock(mtx);
        executePath.header.frame_id = "map";
        executePath.header.stamp = ros::Time::now();

        int size = executePath.poses.size();
        if (size <= 2)
        {
            pubExecutePath.publish(executePath);
            return;
        }

        // truncate path
        int min_id = -1;
        float min_dist = FLT_MAX;

        for (int i = 0; i < size; ++i)
        {
            PointType p;
            p.x = executePath.poses[i].pose.position.x;
            p.y = executePath.poses[i].pose.position.y;
            p.z = robotPoint.z;

            float dist = pointDistance(p, robotPoint);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_id = i;
            }
        }

        if (min_id >= 0 && min_dist < 1.0)
            executePath.poses.erase(executePath.poses.begin(), executePath.poses.begin() + min_id);

        pubExecutePath.publish(executePath);
    }

    bool needNewPath()
    {
        bool needFlag = false;
        // no path available
        if (searchedPath.poses.size() <= 1)
            needFlag = true;
    
        //ImageMapshow = ImageMap.clone();
        //path in collision
        clock_t start,end;
        start=clock();
        for (int i = 0; i < executePath.poses.size()-4; ++i)
        {
            float x = executePath.poses[i].pose.position.x;
            float y = executePath.poses[i].pose.position.y;
            float yaw = tf::getYaw(executePath.poses[i].pose.orientation);

            if (isIncollision_Byimage(x, y, yaw))// || isCloseCollision(x, y))
            {
                ROS_WARN("Obstacles on path, re-planning.");
                needFlag = true;
                break;
            } 
        }
        end=clock();
        double endtime=(double)(end-start)/CLOCKS_PER_SEC;
        // cout<<"collision time:"<<endtime*1000<<"ms"<<endl;	

        if (needFlag == true)
            return true;

        return false;
    }

    void updatePath(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (getRobotPosition() == false) return;

        if (needNewPath()&&receive_imagemap)
        {   
            // buildAdjacencyMatrix();
            // connectAdjacencyMatrix();          
            // searchAdjacencyMatrix();
            // plan();            

            ///bilihyrid
            clock_t start,end;
            start=clock();
            openPlannerRollOut.run(transform, globalPathMessage, globalPath, searchedPath, remainingPath);
            Vect_3d start_state = Vect_3d(
                (double)robotPoint.x,
                (double)robotPoint.y,
                (double)robotPoint.intensity
            );
            Vect_3d goal_state = Vect_3d(
                (double)robotPoint.x,
                (double)robotPoint.y,
                (double)robotPoint.intensity
                );

            if(remainingPath.poses.size()>0)
            {   
                bool need_stop = true;
                int n = remainingPath.poses.size();
                int i = 0;
                while(i<n)
               {
                   if(isIncollision_Byimage(remainingPath.poses[i].pose.position.x, remainingPath.poses[i].pose.position.y, 
                                            tf::getYaw(remainingPath.poses[i].pose.orientation))) i+=10;
                                            else 
                                            {   
                                                need_stop = false;
                                                break;
                                            }
               }
               if(need_stop)
               {
                ROS_WARN("Obstacles on goal, stop!");
                 goal_state = Vect_3d(
                (double)robotPoint.x,
                (double)robotPoint.y,
                (double)robotPoint.intensity
                );
               }
               else
               {
                 goal_state = Vect_3d(
                (double)remainingPath.poses[i].pose.position.x,
                (double)remainingPath.poses[i].pose.position.y,
                (double)NormalizeAngle_pi(tf::getYaw(remainingPath.poses[i].pose.orientation))
                );
               }
               
            }        

            if (kinodynamic_Search(start_state, goal_state))
            {
                ROS_INFO("re-plannig success!");
                auto path = GetPath();
                save_KinoPath(path);
                Publish_imageshow(path);
            }
            end=clock();
            double endtime=(double)(end-start)/CLOCKS_PER_SEC;
            cout<<"Total time0:"<<endtime*1000<<"ms"<<endl;
            Hybrid_Reset();
            end=clock();
            endtime=(double)(end-start)/CLOCKS_PER_SEC;
            cout<<"Total time:"<<endtime*1000<<"ms"<<endl;
            // //bilibli  hybrid


            // visualization();
        }
        visualization();
        // publishPath();
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "lexicographic_planning");
    
    PathPlanning pp;

    ROS_INFO("\033[1;32m----> lexicographic_planning: Path Planning Started.\033[0m");

    ros::spin();

    return 0;
}