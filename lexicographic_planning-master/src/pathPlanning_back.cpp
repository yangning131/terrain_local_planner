#include "utility.h"
#include <kdtree1/kdtree.h>
//#include "planner.h"
#include "constants.h"
#include "node3d.h"
#include "visualize.h"
#include "path.h"
#include "collisiondetection.h"
#include <boost/heap/binomial_heap.hpp>


using namespace cv;
//using namespace VisualizeNS;
class PathPlanning : public ParamServer
{
   public:

    std::mutex mtx;
    ros::Timer pathUpdateTimer;

    ros::Subscriber subGlobalPath;
    ros::Subscriber subObstacleMap;

    ros::Subscriber subImage_collision;
    ros::Subscriber subImage_cost;
    ros::Subscriber subImage_pathmap;
    ros::Subscriber subImage_pathmapBound;
    ros::Publisher  pubImage_robot;

    ros::Publisher pubExecutePath;
    ros::Publisher pubSearchedPath;
    ros::Publisher pubPRMGraph;
    ros::Publisher pubSingleSourcePaths;

    ros::Publisher pub_delete_openplanner;

    nav_msgs::OccupancyGrid occupancyMap2D;

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
    std::vector<Node3D> path_node;

    RolloutGeneratorNS::RolloutGenerator openPlannerRollOut;

    InitChassisDescriptor Chassis;
    cv::Mat ImageMap;
    bool receive_imagemap = false;
    cv::Mat ImageMapCost;

    cv::Mat ImagePathMap;
    cv::Mat ImagePathMapBound;
    

    PathPlanning()
    {   
        subGlobalPath = nh.subscribe<nav_msgs::Path>("planning/server/path_blueprint_smooth", 5, &PathPlanning::pathHandler, this);
        subObstacleMap = nh.subscribe<nav_msgs::OccupancyGrid>("planning/obstacle/map_inflated", 5, &PathPlanning::mapHandler, this);

        subImage_collision = nh.subscribe<sensor_msgs::Image> ("planning/obstacle/ImageMap", 1,&PathPlanning::ImageMapCallback, this);
        subImage_cost = nh.subscribe<sensor_msgs::Image> ("planning/obstacle/ImageMapCost", 1,&PathPlanning::ImageMapCostCallback, this);

        subImage_pathmap = nh.subscribe<sensor_msgs::Image> ("planning/Image_pathmap", 1,&PathPlanning::ImagePathMapCallback, this);
        subImage_pathmapBound = nh.subscribe<sensor_msgs::Image> ("planning/Image_pathmapBound", 1,&PathPlanning::ImagePathMapBoundCallback, this);
        


        pubPRMGraph = nh.advertise<visualization_msgs::MarkerArray>("planning/planning/prm_graph", 5);
        pubSingleSourcePaths = nh.advertise<visualization_msgs::MarkerArray>("planning/planning/prm_single_source_paths", 5);

        pub_delete_openplanner = nh.advertise<visualization_msgs::MarkerArray>("planning/planning/open_planner", 5);

        pubExecutePath = nh.advertise<nav_msgs::Path>("planning/planning/execute_path", 1);
        pubSearchedPath = nh.advertise<nav_msgs::Path>("planning/planning/searched_path", 1);

        pubImage_robot  = nh.advertise<sensor_msgs::Image>("planning/obstacle/Imagerobot", 1);

        ImageMap.release();
        ImageMap = cv::Mat::zeros(400, 400, CV_8UC1);

        ImageMapCost.release();
        ImageMapCost = cv::Mat::zeros(400, 400, CV_8UC1);

        ImagePathMap.release();
        ImagePathMap = cv::Mat::zeros(400, 400, CV_8UC1);

        ImagePathMapBound.release();
        ImagePathMapBound = cv::Mat::zeros(400, 400, CV_8UC1);





        adjacency_width_grid = -1;
        adjacency_length_grid = -1;

        pathUpdateTimer = nh.createTimer(ros::Duration(1.0/5.0), &PathPlanning::updatePath, this);
    }

    void ImageMapCallback (const sensor_msgs::ImageConstPtr& imag)
    {
        // std::lock_guard<std::mutex> lock(mtx);

        // localMapFrame_ = image->header.frame_id;

        // ros::Time mapTime = image->header.stamp;

        // std::string map_frame = image->header.frame_id;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imag, sensor_msgs::image_encodings::TYPE_8UC1);
	    ImageMap = cv_ptr -> image;

        // ImageMap = cv_bridge::toCvShare(imag,sensor_msgs::image_encodings::TYPE_8UC1)->image;
        receive_imagemap = true;
    }
    void ImageMapCostCallback (const sensor_msgs::ImageConstPtr& imag)
    {
         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imag, sensor_msgs::image_encodings::TYPE_8UC1);
	     ImageMapCost = cv_ptr -> image;
    }
    void ImagePathMapCallback (const sensor_msgs::ImageConstPtr& imag)
    {
         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imag, sensor_msgs::image_encodings::TYPE_8UC1);
	     ImagePathMap = cv_ptr -> image;
    }

    void ImagePathMapBoundCallback (const sensor_msgs::ImageConstPtr& imag)
    {
         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imag, sensor_msgs::image_encodings::TYPE_8UC1);
	     ImagePathMapBound = cv_ptr -> image;

    }

    int GetChassisdescriptorIndex(const float& theta)
    {    
        float angle = Chassis.NormalizeAngle(theta) *180/PI_;
        return Chassis.GetAngleIdxFast(angle);
    }

    



    void pathHandler(const nav_msgs::Path::ConstPtr& pathMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (pathMsg->poses.size() <= 1)
        {
            ROS_WARN("Empty global path received.");
            return;
        }

        globalPathMessage = *pathMsg;

        // treate the original centerPath from open_planner as searched path
        openPlannerRollOut.run(transform, globalPathMessage, globalPath, searchedPath, remainingPath, alternativePaths);
        executePath = combinePaths(searchedPath, remainingPath);
        pubSearchedPath.publish(executePath);

        // subGlobalPath.shutdown();
        ROS_INFO("\033[1;32m Global Path recieved. \033[0m");
    }

    void mapHandler(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
       
       configurationSpace._mapResolution=_mapResolution;
        occupancyMap2D = *mapMsg;
        configurationSpace.updateGrid(occupancyMap2D);
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
      if (nSucc->isInRange(goal)  || iterations > Constants::iterations) {
        // DEBUG
        std::cout<<"iterations:"<<iterations<<std::endl;
        visualization_.publishNode3DPose(*nSucc);
        return nSucc;
      }

        //     // ____________________
       //     // CONTINUE WITH SEARCH
      else {
        // _______________________
        // SEARCH WITH DUBINS SHOT
        if (nSucc->isInRange(goal) ) {

              std::cout<<"iterations22:::"<<iterations<<std::endl;
          //nSucc = dubinsShot(*nPred, goal, configurationSpace);

           //if (nSucc != nullptr && *nSucc == goal) {
            //DEBUG
             // std::cout << "max diff " << max << std::endl;
             visualization_.publishNode3DPose(*nSucc);
               return nSucc;
          //  }
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
           if (nSucc->isOnGrid(width, height,occupancyMap2D.info.origin.position.x, occupancyMap2D.info.origin.position.y) && configurationSpace.isTraversable(nSucc)&&isongraph(nSucc)) {
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
void tracePath(const Node3D* node, int i=0, std::vector<Node3D> path_node=std::vector<Node3D>()) {
  if (node == nullptr) {
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
       std::cout<<"oalState->theta;="<<t<<std::endl;
      //   // set theta to a value (0,2PI]

       t = Helper::normalizeHeadingRad(t);

        std::cout<<"oalState-a;="<<t<<std::endl;
     const Node3D nGoal(x, y, t, 0, 0, nullptr);
     Node3D vi=nGoal;
     visualization_.publishNode3DPose_st(vi);

      //    // __________
      //     // DEBUG GOAL
      //    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);
     

    //  // _________________________
    //  // retrieving start position
   float x_ =startState->x / Constants::cellSize;
    float y_ = startState->y / Constants::cellSize;
   float t_ = startState->theta;
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
       path.updatePath_(path_node);
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
            if (pubImage_robot.getNumSubscribers() !=0&&receive_imagemap)
          {
            cv::Mat robot_thata;


            //robot_thata.release();
            //  robot_thata = cv::Mat::zeros(150, 150, CV_16UC1);
            // cv::cvtColor(robot_thata,robot_thata,cv::COLOR_BGR2GRAY);
             std::cout<<"index test: "<<GetChassisdescriptorIndex(robotPoint.intensity)<<std::endl;

            robot_thata = Chassis.descriptors[GetChassisdescriptorIndex(robotPoint.intensity)].image_collision;//.. 

            // cout<<"Chassis.descriptors[90].image_or;"<<Chassis.descriptors[90].image_or.type()<<endl;
             cv_bridge::CvImage out_z_image;
            //  out_z_image.header   = laserCloudMsg->header; // Same timestamp and tf frame as input image
             out_z_image.header.stamp   = ros::Time::now();// Same timestamp and tf frame as input image
             out_z_image.header.frame_id   = "map"; // Same timestamp and tf frame as input image
            //  if (robot_thata.type() == CV_32F)
            //    {
            //        out_z_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
            //    }
            //  else
            //     {
            //         out_z_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // Or whatever
            //     }
            out_z_image.encoding = "mono8";
                //robot_thata = cv::Scalar(255,255) - robot_thata;
             out_z_image.image    = robot_thata; // Your cv::Mat

              pubImage_robot.publish(out_z_image.toImageMsg());

               

                   // cv::Mat robot_thata = Chassis.descriptors[GetChassisdescriptorIndex(yaw)].image_;
              

            int row = ImageMap.rows;
           int col = ImageMap.cols;
        //    cv::cvtColor(ImageMap,ImageMap,cv::COLOR_BGR2GRAY);
        //    cv::cvtColor(robot_thata,robot_thata,cv::COLOR_BGR2GRAY);
        //    robot_thata = cv2::cvtColor(robot_thata,cv2::COLOR_BGR2GRAY)

    // if (ImageMap.channels() == 3)
    // {
    //     cv::cvtColor(ImageMap,ImageMap,cv::COLOR_BGR2GRAY);
    // }
    // if (ImageMap.channels() == 4)
    // {
    //     cv::cvtColor(ImageMap,ImageMap,cv::COLOR_BGRA2GRAY);
    // }


    // if (robot_thata.channels() == 3)
    // {
    //     cv::cvtColor(robot_thata,robot_thata,cv::COLOR_BGR2GRAY);
    // }
    // if (robot_thata.channels() == 4)
    // {
    //     cv::cvtColor(robot_thata,robot_thata,cv::COLOR_BGRA2GRAY);
    // }


    // std::cout<<"dot test: "<<robot_thata.rows<<" -----"<<robot_thata.cols<<std::endl;
    cv::Mat imageROI = ImageMap(Rect((col-robot_thata.cols)/2,(row-robot_thata.rows)/2,robot_thata.cols,robot_thata.rows));
        //   cv::cvtColor(imageROI,imageROI,cv::COLOR_BGR2GRAY);
        //   robot_thata.convertTo(robot_thata, CV_8UC1);
        //   imageROI.convertTo(imageROI, CV_8UC1);
          std::cout<<"imageROI.rows: "<<imageROI.rows<<"imageROI.cols: "<<imageROI.cols<<" index"<<ImageMap.at<int>(100, 100)<<std::endl;
          std::cout<<"imageROI.rows: "<<robot_thata.rows<<"imageROI.cols: "<<robot_thata.cols<<" index"<<robot_thata.at< int>(10, 10)<<std::endl;
        std::cout<<"imageROI.type: "<<imageROI.type()<<std::endl;
                std::cout<<"imageROI.type: "<<robot_thata.type()<<std::endl;


        // // imageROI = cv::Scalar(255,255) - imageROI;
           int test = robot_thata.dot(imageROI);
           std::cout<<"dot test: "<<test<<std::endl;
           receive_imagemap  = false;
          }

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
        robotPoint.intensity = yaw;

        if (globalPathMessage.poses.size() == 0)
            return false;

        if (occupancyMap2D.data.size() == 0)
            return false;

        lastRobotPoint = robotPoint;
       


        

        return true;
    }

    void publishPath()
    {
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
        
        // path in collision
        for (int i = 0; i < executePath.poses.size(); ++i)
        {
            float x = executePath.poses[i].pose.position.x;
            float y = executePath.poses[i].pose.position.y;

            if (isIncollision(x, y))// || isCloseCollision(x, y))
            {
                ROS_WARN("Obstacles on path, re-planning.");
                needFlag = true;
                break;
            } 
        }

        if (needFlag == true)
            return true;

        return false;
    }

    void updatePath(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (getRobotPosition() == false) return;

        //std::cout<<"Chassis.descriptors[90].image_or;"<<Chassis.descriptors[90].image_or.type()<<std::endl;


        if (needNewPath())
        {
            buildAdjacencyMatrix();

            connectAdjacencyMatrix();
            
            searchAdjacencyMatrix();
            plan();
            visualization();
        }
        visualization();

        publishPath();
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "lexicographic_planning");
    
    PathPlanning pp;

    ROS_INFO("\033[1;32m----> lexicographic_planning: Path Planning Started.\033[0m");

    ros::spin();

    return 0;
}