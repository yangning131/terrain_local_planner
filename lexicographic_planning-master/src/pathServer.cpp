#include "utility.h"


class PathServer : public ParamServer
{
public:

    ros::Timer pathUpdateTimer;

    ros::Publisher pubPathRaw;
    ros::Publisher pubPathSmooth;

    ros::Subscriber  mygoal_;
    PointType mygoal;

    nav_msgs::Path pathRaw;
    nav_msgs::Path pathSmooth;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    PointType robotPoint;

    double radius = 2;  
    double length = 2;
    double width = radius * 2;

    bool receive_goal;

    PathServer()
    {
        
         pubPathRaw = nh.advertise<nav_msgs::Path> ("planning/server/path_blueprint_raw", 1);
        pubPathSmooth = nh.advertise<nav_msgs::Path> ("planning/server/path_blueprint_smooth", 1); 
        //   pubPathSmooth = nh.advertise<nav_msgs::Path> ("planning/planning/execute_path", 1); 
        receive_goal =false;
        mygoal_=nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2, &PathServer::submygoal,this);
        // receive_goal =false;
        //    pathUpdateTimer = nh.createTimer(ros::Duration(10.0), &PathServer::updatePath, this);
        // updatePath();

      
    };


    void submygoal(const geometry_msgs::PoseStamped::ConstPtr& mygoal_)
    {  geometry_msgs::PoseStamped pt = *mygoal_;
        mygoal.x  =pt.pose.position.x;
        mygoal.y=pt.pose.position.y;
        mygoal.z=0;
        // std::cout<<"mygoal.x"<<mygoal.x<<"mygoal.y"<<mygoal.y<<endl;
     //   createPath1();
   //     updatePath();
   receive_goal = true;
     updatePath();
    }

    bool getRobotPosition()
    {
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return false; }
        
        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = 0;

        return true;
    }

    void createPath1()
    {
         getRobotPosition();
        pathRaw = nav_msgs::Path();
        // create raw path


        pathRaw.poses.push_back(createPoseStamped(robotPoint.x , robotPoint.y, 0));
        // pathRaw.poses.push_back(createPoseStamped(0,0, 0));


      //  pathRaw.poses.push_back(createPoseStamped(1, 1, 0));
        pathRaw.poses.push_back(createPoseStamped(mygoal.x, mygoal.y, mygoal.z));

        // for (double angle = 0; angle <= M_PI; angle += M_PI / 18)
        // {
        //     float x = length + radius * sin(angle);
        //     float y = width/2 - radius * cos(angle);
        //     pathRaw.poses.push_back(createPoseStamped(x, y, 0));
        // }

        // pathRaw.poses.push_back(createPoseStamped(length, width, 0));
        // pathRaw.poses.push_back(createPoseStamped(0, width, 0));

        // smooth path
        pathSmooth = processPath(pathRaw);
    }
  

    void createPath2()
    {
        pathRaw = nav_msgs::Path();
        // create raw path
        pathRaw.poses.push_back(createPoseStamped(0, width, 0));
        pathRaw.poses.push_back(createPoseStamped(-length, width, 0));

        // for (double angle = M_PI; angle <= 2 * M_PI; angle += M_PI / 18)
        // {
        //     float x = -length + radius * sin(angle);
        //     float y =  width/2 - radius * cos(angle);
        //     pathRaw.poses.push_back(createPoseStamped(x, y, 0));
        // }

        pathRaw.poses.push_back(createPoseStamped(-length, 0, 0));
        pathRaw.poses.push_back(createPoseStamped(0, 0, 0));

        // smooth path
        pathSmooth = processPath(pathRaw);
    }


    geometry_msgs::PoseStamped createPoseStamped(float x, float y, float z)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x;
        pose.pose.position.y = y; 
        pose.pose.position.z = z;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        return pose;
    }

    void publishGlobalPath()
    {
        if (pubPathRaw.getNumSubscribers() != 0)
        {
            pathRaw.header.frame_id = "map";
            pathRaw.header.stamp = ros::Time::now();
            pubPathRaw.publish(pathRaw);
        }
        
        if (pubPathSmooth.getNumSubscribers() != 0)
        {
            pathSmooth.header.frame_id = "map";
            pathSmooth.header.stamp = ros::Time::now();
            pubPathSmooth.publish(pathSmooth);
        }
    }

  //  void updatePath(const ros::TimerEvent& event)
    void updatePath()
    {
        if (getRobotPosition() == false) return;

        PointType p1;
        p1.x = 0;
        p1.y = 0;
        p1.z = 0;

        PointType p2;
        p2.x = 0;
        p2.y = width;
        p2.z = 0;

        // if (pointDistance(robotPoint, p1) < 1.0)
        //     createPath1();
        // else if (pointDistance(robotPoint, p2) < 1.0)
        //     createPath2();
        // else
        //     return;

        if(receive_goal)
       createPath1();
       
        publishGlobalPath();
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "lexicographic_planning");
    
    PathServer ps;

    ROS_INFO("\033[1;32m----> lexicographic_planning: Path Server Started.\033[0m");

    ros::spin();

    return 0;
}