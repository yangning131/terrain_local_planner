/**
 * @file collisiondetection.cpp
 * @brief 碰撞检测函数集
 * @date 2019-12-12
 * 
 * 
 */
#include "collisiondetection.h"



CollisionDetection::CollisionDetection() {
  //this->grid = nullptr;
  Lookup::collisionLookup(collisionLookup);
}

bool CollisionDetection::configurationTest(float x, float y, float t) {
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * Constants::positionResolution);//得出X方向在cell中的偏移量
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * Constants::positionResolution);//Y方向在cell中的偏移量
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / Constants::deltaHeadingRad);
  int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  int cX;
  int cY;
    //std::cout<<"grid: "<<grid.info.origin.position.x<<std::endl;
  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

       int index_x = (int)round((cX - grid.info.origin.position.x) / _mapResolution);
        int index_y = (int)round((cY - grid.info.origin.position.y) / _mapResolution);
        int index = index_x + index_y * grid.info.width;  
  

    // make sure the configuration coordinates are actually on the grid
    if (index_x >= 0 && index_x < grid.info.width && index_y >= 0 && index_y < grid.info.height) {
      if (grid.data[index]==100) {
        return false;
      }//若grid的某个小网格存在值，说明有障碍，则返回false表示不在自由网格
    }
  }

  return true;//所有检测都没有检测到被占用，说明没有障碍，可以通行
}




        //    int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
        // int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);
        // int index = index_x + index_y * occupancyMap2D.info.width;

        // if (index_x < 0 || index_x >= occupancyMap2D.info.width ||
        //     index_y < 0 || index_y >= occupancyMap2D.info.height)
        //     return false;




        // int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
        // int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);
        // int index = index_x + index_y * occupancyMap2D.info.width;

        // if (index_x < 0 || index_x >= occupancyMap2D.info.width ||
        //     index_y < 0 || index_y >= occupancyMap2D.info.height)
        //     return false;

        // //if (occupancyMap2D.data[index] == 100)
        //   if (occupancyMap2D.data[index] >0)
        //     return true;
        // else
        //     return false;