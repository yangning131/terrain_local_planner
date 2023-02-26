#include "utility.h"

class pathmap : public ParamServer
{
public:
    std::mutex mtx;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber subGlobalPath;

   

    ros::Publisher pubOccupancyMap;
    ros::Publisher pubOccupancyMap2;
    nav_msgs::Path globalPathMessage;

    nav_msgs::Path nav_path_1;
    nav_msgs::Path nav_path_2;
    nav_msgs::Path nav_path_3;
     
    vector<nav_msgs::Path>  rolloutpaths;

   
    deque<double> timeQueue;

    nav_msgs::OccupancyGrid occupancyMap2D;

    int count = 0;

    float _mapResolution ;

    int  _local_map_grid_num;
    float _local_map_length;

    ros::Publisher pubImage_pathmap;
    ros::Publisher pubImage_pathmapBound;
    cv::Mat img_mat_;
    cv::Mat img_mat_bound;

    


    pathmap()
    {

        subGlobalPath = nh.subscribe<nav_msgs::Path>("expath222", 5, &pathmap::pathHandler, this);

        pubOccupancyMap  = nh.advertise<nav_msgs::OccupancyGrid> ("planning/pathmap_path", 1);

        pubImage_pathmap  = nh.advertise<sensor_msgs::Image> ("planning/Image_pathmap", 1);

        pubImage_pathmapBound  = nh.advertise<sensor_msgs::Image> ("planning/Image_pathmapBound", 1);

        
        initializePathMap();
    }


        // inline void ResetImgMat(cv::Mat& img_mat) 
        // {
        //             img_mat.release();
        //     img_mat = cv::Mat::zeros(_local_map_grid_num, _local_map_grid_num, CV_8UC1);
        // }
         void ResetImgMatBound(cv::Mat& img_mat) 
        {
            img_mat.release();  
            //  cv::Mat mat(_local_map_grid_num,_local_map_grid_num,CV_8UC1,Scalar(254));
            // img_mat = mat;
            img_mat = cv::Mat::zeros(_local_map_grid_num, _local_map_grid_num, CV_32FC1);
            // img_mat = img_mat +  cv::Scalar(255);
    
            for(int i=0;i<_local_map_grid_num;i++)
                for(int j =0 ;j<_local_map_grid_num;j++)
                 {
                        img_mat.at<float>(i,j) = 254;
                 }
            
                        std::cout<<"img_mat row :"<<img_mat.rows<<std::endl;
                        std::cout<<"img_mat col :"<<img_mat.cols<<std::endl;
                        std::cout<<"_local_map_grid_num :"<<_local_map_grid_num<<std::endl;
           
        }

        inline void ResetImgMat(cv::Mat& img_mat) {
        img_mat.release();
        img_mat = cv::Mat::zeros(_local_map_grid_num, _local_map_grid_num, CV_32FC1);
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

    void pathHandler(const nav_msgs::Path::ConstPtr& pathMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (pathMsg->poses.size() <= 1)
        {
            ROS_WARN("Empty global map path received.");
            return;
        }

        globalPathMessage = *pathMsg;
        

          rolloutpaths.clear();
          rolloutpaths.resize(_rollOutNumber+1);

       

     if (globalPathMessage.poses.size() < 2)
          ;
      else if (globalPathMessage.poses.size() == 2) {
        globalPathMessage.poses[0].pose.position.z  = cast_from_0_to_2PI_Angle(atan2(globalPathMessage.poses[1].pose.position.y -globalPathMessage.poses[0].pose.position.y, globalPathMessage.poses[1].pose.position.x - globalPathMessage.poses[0].pose.position.x));
        globalPathMessage.poses[1].pose.position.z = globalPathMessage.poses[0].pose.position.z;
        
    }
    else
    {
     
      globalPathMessage.poses[0].pose.position.z = cast_from_0_to_2PI_Angle(atan2(globalPathMessage.poses[1].pose.position.y -globalPathMessage.poses[0].pose.position.y, globalPathMessage.poses[1].pose.position.x - globalPathMessage.poses[0].pose.position.x));
    

     for (int j = 1; j < globalPathMessage.poses.size() - 1; j++) {
        globalPathMessage.poses[j].pose.position.z  = cast_from_0_to_2PI_Angle(atan2(globalPathMessage.poses[j + 1].pose.position.y - globalPathMessage.poses[j].pose.position.y, globalPathMessage.poses[j + 1].pose.position.x - globalPathMessage.poses[j].pose.position.x));
        
     }

     int j = (int)globalPathMessage.poses.size() - 1;
     globalPathMessage.poses[j].pose.position.z =  globalPathMessage.poses[j-1].pose.position.z;
    }

       int length = globalPathMessage.poses.size();

       geometry_msgs::PoseStamped pose_stamped;
       pose_stamped.header.frame_id = "map";
       for (int i = 0; i < length - 1; ++i)
       {

         for(unsigned int j = 0; j < _rollOutNumber ; j++)
         {
             if(j<_rollOutNumber/2)
             {
            pose_stamped.pose.position.x = globalPathMessage.poses[i].pose.position.x+_pathResolution*(j)*std::sin(globalPathMessage.poses[i].pose.position.z);
            pose_stamped.pose.position.y =globalPathMessage.poses[i].pose.position.y-_pathResolution*(j)*std::cos(globalPathMessage.poses[i].pose.position.z);
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(globalPathMessage.poses[i].pose.position.z);
             }
             else{
                 pose_stamped.pose.position.x = globalPathMessage.poses[i].pose.position.x-_pathResolution*(j-_rollOutNumber/2)*std::sin(globalPathMessage.poses[i].pose.position.z);
                 pose_stamped.pose.position.y =globalPathMessage.poses[i].pose.position.y+_pathResolution*(j-_rollOutNumber/2)*std::cos(globalPathMessage.poses[i].pose.position.z);
                 pose_stamped.pose.position.z = 0.0;
                 pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(globalPathMessage.poses[i].pose.position.z);

             }

             rolloutpaths[j].poses.emplace_back(pose_stamped);
            
         }
        
        }

       
       for(int i = 0 ;i<_rollOutNumber ; i++)
       {
        processPath_pathmap(rolloutpaths.at(i));   
       }

         

        try{listener.lookupTransform("map","base_link", ros::Time(0), transform);} 
        catch (tf::TransformException ex){ return; }

        // double timeScanCur = laserCloudMsg->header.stamp.toSec();

        PointType robotPoint;
        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = transform.getOrigin().z();


          if (pubOccupancyMap.getNumSubscribers() != 0||pubImage_pathmap.getNumSubscribers() > 0||pubImage_pathmapBound.getNumSubscribers() > 0)
        {   
            ResetImgMat(img_mat_);
            ResetImgMatBound(img_mat_bound);
            
            // occupancyMap2D.header.stamp = laserCloudMsg->header.stamp;
            occupancyMap2D.info.origin.position.x = robotPoint.x - _local_map_length / 2.0;
            occupancyMap2D.info.origin.position.y = robotPoint.y - _local_map_length / 2.0;
            occupancyMap2D.info.origin.position.z = -0.1;
            std::fill(occupancyMap2D.data.begin(), occupancyMap2D.data.end(), 0);

           


                 for (int j = 0; j < _rollOutNumber; ++j)
            {

                for (int i = 0; i < rolloutpaths.at(j).poses.size(); i++)
                {
                   int index_x = ( rolloutpaths.at(j).poses[i].pose.position.x - occupancyMap2D.info.origin.position.x) / _mapResolution;
                   int index_y = ( rolloutpaths.at(j).poses[i].pose.position.y - occupancyMap2D.info.origin.position.y) / _mapResolution;
                   if (index_x < 0 || index_y < 0 || index_x >= _local_map_grid_num || index_y >= _local_map_grid_num)
                            continue;

                        int index = index_y * occupancyMap2D.info.width + index_x;
                        if(j<_rollOutNumber/2)
                        {
                     
                          occupancyMap2D.data[index] = j*5;
                          img_mat_.at<float>(index_x, index_y) = j*5;
                          img_mat_bound.at<float>(index_x, index_y) = 0;
                        }
                        else{
                            occupancyMap2D.data[index] = (j-_rollOutNumber/2)*5;
                            img_mat_.at<float>(index_x, index_y) = (j-_rollOutNumber/2)*5;
                           img_mat_bound.at<float>(index_x, index_y) = 0;
                        }
                        
                }
                
               
            }


         if (pubImage_pathmap.getNumSubscribers() > 0)
          { 
              
              cv::Size dsize = cv::Size(400,400);
              cv::Mat imgtemp = cv::Mat(dsize,CV_32FC1);
              cv::resize(img_mat_,imgtemp,dsize);
             cv_bridge::CvImage out_z_image;
            //  out_z_image.header   = dlaserCloudMsg->header; // Same timestamp and tf frame as input image
             out_z_image.header.stamp   = ros::Time::now();; // Same timestamp and tf frame as input image
             out_z_image.header.frame_id   = "map"; // Same timestamp and tf frame as input image
            //   out_z_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
             if (imgtemp.type() == CV_32F)
               {
                   out_z_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
               }
             else
                { 
                    out_z_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // Or whatever
                }
             out_z_image.image    = imgtemp; // Your cv::Mat
             pubImage_pathmap.publish(out_z_image.toImageMsg());
          }

         if (pubImage_pathmapBound.getNumSubscribers() > 0)
          {

               cv::Size dsize = cv::Size(400,400);
              cv::Mat imgtemp = cv::Mat(dsize,CV_32FC1);
              cv::resize(img_mat_bound,imgtemp,dsize);

             cv_bridge::CvImage out_z_image;
            //  out_z_image.header   = dlaserCloudMsg->header; // Same timestamp and tf frame as input image
             out_z_image.header.stamp   = ros::Time::now();; // Same timestamp and tf frame as input image
             out_z_image.header.frame_id   = "map"; // Same timestamp and tf frame as input image
             if (imgtemp.type() == CV_32F)
               {
                   out_z_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
               }
             else
                {
                    out_z_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // Or whatever
                }

             out_z_image.image    = imgtemp; // Your cv::Mat
             pubImage_pathmapBound.publish(out_z_image.toImageMsg());
              cout<<"img_mat_bound*********************  "<<imgtemp.at<float>(10,10)<<endl;
            //    cout<<"img_mat_bound*********************  "<<img_mat_bound<<endl;
          }


         pubOccupancyMap.publish(occupancyMap2D);
         ROS_INFO("\033[1;32m Global Map Path recieved. \033[0m");
        }
    }


    void initializePathMap()
    {  
        _mapResolution = _pathResolution+0.15;
        _local_map_grid_num = round((_sensorRangeLimitMax+4) * 4.0 / _mapResolution);//*4.0
        _local_map_length = (_sensorRangeLimitMax+4 )* 4.0;//*4.0
        occupancyMap2D.header.frame_id = "map";
        occupancyMap2D.info.width = _local_map_grid_num;
        occupancyMap2D.info.height = _local_map_grid_num;
        occupancyMap2D.info.resolution = _mapResolution;
        occupancyMap2D.info.origin.orientation.x = 0.0;
        occupancyMap2D.info.origin.orientation.y = 0.0;
        occupancyMap2D.info.origin.orientation.z = 0.0;
        occupancyMap2D.info.origin.orientation.w = 1.0;
        occupancyMap2D.data.resize(occupancyMap2D.info.width * occupancyMap2D.info.height);
    } 


   
 };


int main(int argc, char** argv){

    ros::init(argc, argv, "lexicographic_planning");
    
    pathmap os;

    ROS_INFO("\033[1;32m----> lexicographic_planning: pathmap Server Started.\033[0m");

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }

    ros::spin();

    return 0;
}