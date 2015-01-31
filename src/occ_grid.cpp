#include <csm_utils/occ_grid.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
#define VAL_UNKNOWN -1
#define VAL_EVEN 50
#define VAL_MAX 100
#define VAL_MIN 2
#define OCC_THR 90
#ifndef MAX
#define MAX(a,b) (a>b)?a:b
#endif
#ifndef MIN
#define MIN(a,b) (a<b)?a:b
#endif

namespace laser_slam{
  OccGrid::OccGrid() {
    origin_.x = 0; 
    origin_.y = 0;
    dim_.x = 0; 
    dim_.y = 0;
    expand_dim_ = 5;
    res = 0.05;
  }

  OccGrid::~OccGrid() {
  }

  bool OccGrid::GetCell(int2 pose, char*& cell) 
  {
    if(pose.x < 0 || pose.y < 0 ||
        pose.x >= dim_.x || pose.y >= dim_.y)
      return false;
    cell = &(map[pose.x][pose.y]);
    return true;
  }


  void OccGrid::
    SetRes(double res_) {
      res = res_;
      printf("OccGrid: resolution = %.3f", res);
    }

  bool OccGrid::Allocate(int2 new_dim, int2 new_ori)
  {
    if (dim_.x == new_dim.x &&
        dim_.y == new_dim.y &&
        origin_.x == new_ori.x &&
        origin_.y == new_ori.y)
      return false;
    else {
      //reallocate, and copy
      array_type newmap(boost::extents[new_dim.x][new_dim.y]);

      for (int w = 0; w < new_dim.x; w++) {
        for (int h = 0; h < new_dim.y; h++){
          if (w + new_ori.x < origin_.x || 
              h + new_ori.y < origin_.y ||
              w + new_ori.x >= origin_.x + dim_.x ||
              h + new_ori.y >= origin_.y + dim_.y)
            newmap[w][h] = VAL_UNKNOWN;
          else {
            newmap[w][h] = map[w + new_ori.x - origin_.x][h + new_ori.y - origin_.y];
          }
        }
      }
      map.resize(boost::extents[new_dim.x][new_dim.y]);
      map = newmap;
      dim_ = new_dim;
      origin_ = new_ori;
    }
    return true;
  }

  void OccGrid::Clear() 
  {
    for (int w = 0; w < dim_.x; w++) 
      for (int h = 0; h < dim_.y; h++) 
        map[w][h] = VAL_UNKNOWN;
  }

  void IncSafe(char& val) {
   val = VAL_MAX;
   return;
    if (val == VAL_UNKNOWN)
      val = VAL_MAX;
    else {
      if (val < VAL_MAX-29)
        val += 30;
      else  val = VAL_MAX;
    }
  }

  void DecSafe(char& val) {
    if (val == VAL_UNKNOWN)
      val = VAL_EVEN - 2;
    else {
      if (val > VAL_MIN)
        val -= 2;
      else
        val = VAL_MIN;
    }

  }


  void OccGrid::AddBeam(int2 start, int2 end) 
  {
    char* cell;
    if (GetCell(end, cell))
      IncSafe(*cell);

    bool steep = abs(end.y - start.y) > abs(end.x - start.x);
    if (steep) {
      int x = start.x;
      start.x = start.y;
      start.y = x;
      int y = end.x;
      end.x = end.y;
      end.y = y;
    }
    if (start.x > end.x) {
      int x = start.x;
      start.x = end.x;
      end.x = x;
      int y = start.y;
      start.y = end.y;
      end.y = y;
    }
    int delta_x = end.x - start.x;
    int delta_y = abs(end.y - start.y);
    int error = delta_x / 2;
    int ystep;
    int y = start.y;
    if (start.y < end.y)
      ystep = 1;
    else
      ystep = -1;
    for ( int x = start.x; x < end.x; x++) 
    {
      int2 test_p;
      if (steep) 
      {
        test_p.x = y;
        test_p.y = x;
      }
      else
      {
        test_p.x = x;
        test_p.y = y;
      }
      if (GetCell(test_p, cell))
        DecSafe(*cell);
      error -= delta_y;
      if (error < 0) {
        y += ystep;
        error += delta_x;
      }
    }

  }

  bool OccGrid::AddLaser(const gtsam::Pose2& pose, const sensor_msgs::PointCloud& cloud) 
  {
    //vote the beams in, clear out along the beam
    
    float co = cos(pose.theta());
    float so = sin(pose.theta());
    int max_x = origin_.x + dim_.x;
    int max_y = origin_.y + dim_.y;
    int min_x = origin_.x;
    int min_y = origin_.y;

    for (size_t i = 0; i < cloud.points.size(); i++) {
      int end_x = (pose.x() + co * cloud.points[i].x - so * cloud.points[i].y) / res;
      int end_y = (pose.y() + so * cloud.points[i].x + co * cloud.points[i].y) / res;

      if (max_x < end_x)
        max_x = end_x;
      else if(end_x < min_x)
        min_x = end_x;
      if(max_y < end_y)
        max_y = end_y;
      else if(end_y < min_y)
        min_y =  end_y;
    }
    int2 new_origin = origin_;
    int2 new_dim = dim_;
    if(min_x < origin_.x)
    {    
      new_origin.x = min_x - expand_dim_;
      new_dim.x += origin_.x - new_origin.x;
    }    
    if(min_y < origin_.y)
    {    
      new_origin.y = min_y - expand_dim_;
      new_dim.y += origin_.y - new_origin.y;
    }    
    if(max_x > origin_.x + dim_.x)
      new_dim.x += expand_dim_ + max_x - (origin_.x + dim_.x);
    if(max_y > origin_.y + dim_.y)
      new_dim.y += expand_dim_ + max_y - (origin_.y + dim_.y);

    bool expanded = Allocate(new_dim, new_origin);

    int2 robot;
    robot.x = pose.x() / res - origin_.x;
    robot.y = pose.y() / res - origin_.y;


    int2 end;
    int2 end_p(-1, -1);
    for (size_t i = 0; i < cloud.points.size(); i++) {
      end.x = (pose.x() + co * cloud.points[i].x - so * cloud.points[i].y) / res - origin_.x;
      end.y = (pose.y() + so * cloud.points[i].x + co * cloud.points[i].y) / res - origin_.y;

       if(end.x != end_p.x || end.y != end_p.y)
          AddBeam(robot, end);
        end_p = end;
    }

    return expanded;
  }

  nav_msgs::GetMap::Response OccGrid::GetMap(const std::string& frame_id) 
  {
    nav_msgs::GetMap::Response map_;
    map_.map.info.resolution = res;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;

    map_.map.info.width = dim_.x;
    map_.map.info.height = dim_.y;

    map_.map.info.origin.position.x = (origin_.x) * res;
    map_.map.info.origin.position.y = (origin_.y) * res;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
    for(int x = 0; x < dim_.x; x++)
      for(int y = 0; y < dim_.y; y++)
      {
        char val = map[x][y];
        if (val == VAL_UNKNOWN)
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = VAL_UNKNOWN;
        else if (val >= OCC_THR)
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
        else 
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;

      }
    map_.map.header.frame_id = frame_id;
    return map_;
  }

  sensor_msgs::PointCloud OccGrid::filterCloud(const sensor_msgs::PointCloud& cloud)
  {
    sensor_msgs::PointCloud cloud_f;
    cloud_f.header = cloud.header;
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
      int x = cloud.points[i].x / res - origin_.x;
      int y = cloud.points[i].y / res - origin_.y;
      if(x >= 0 && x < dim_.x && y >= 0 && y < dim_.y)
        if(map[x][y] >= OCC_THR)
          cloud_f.points.push_back(cloud.points[i]);
    }
    return cloud_f;
  }


  void OccGrid::decayMap(char dec)
  {
    for (int x = 0; x < dim_.x; x ++)
      for (int y = 0; y < dim_.y; y ++)
      {
        if(map[x][y] == VAL_UNKNOWN)
          continue;
        char* cell = &(map[x][y]);
        if(*cell > VAL_EVEN + dec)
          *cell -= dec;
        else if(*cell <= VAL_EVEN + dec && 
            *cell >= VAL_EVEN)
          *cell = VAL_UNKNOWN;
        else if(*cell < VAL_EVEN - dec)
          *cell += dec;
        else if(*cell >= VAL_EVEN - dec &&
            *cell < VAL_EVEN)
          *cell = VAL_UNKNOWN;
      }
  }
}
