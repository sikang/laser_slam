#include <nav_msgs/GetMap.h>
#include <sensor_msgs/PointCloud.h>
#include <gtsam/geometry/Pose2.h>
#include <boost/multi_array.hpp>
#include <stdio.h>
#include <boost/unordered_set.hpp>

namespace laser_slam{

  template <class T>
    class Type2 {
      public:
        T x;
        T y;

        Type2 operator+(const Type2& b)
        {
          Type2 new_c;
          new_c.x = x + b.x;
          new_c.y = y + b.y;
          return new_c;
        }
        Type2 operator-(const Type2& b)
        {
          Type2 new_c;
          new_c.x = x - b.x;
          new_c.y = y - b.y;
          return new_c;
        }
        Type2 operator*(const T& b)
        {
          Type2 new_c;
          new_c.x = x * b;
          new_c.y = y * b;
          return new_c;
        }
        Type2 operator/(const T& b)
        {
          Type2 new_c;
          new_c.x = new_c.x / b;
          new_c.y = new_c.y / b;
          return new_c;
        }

        Type2 (T a=0, T b=0) :
          x(a), y(b){} 
    };  


  class OccGrid {
    public:
      typedef boost::multi_array<char,2> array_type;
      typedef Type2<int> int2;

    protected:
      int2 origin_, dim_;
      double res;
      double expand_dim_;
      array_type map;
    public:
      OccGrid();
      ~OccGrid();
      void SetRes(double res_);
      bool Allocate(int2 new_dim, int2 new_ori);
      void Clear();
      void AddOccupied(int2 end, boost::unordered_set<int> &occupied);
      void AddBeam(int2 start, int2 end, 
          const boost::unordered_set<int>& occupied, 
          boost::unordered_set<int> &ignore);
      bool GetCell(int2 pose, char*& cell);
      bool AddLaser(const gtsam::Pose2& pose, 
          const sensor_msgs::PointCloud& cloud);
      nav_msgs::GetMap::Response GetMap();
      sensor_msgs::PointCloud filterCloud(const sensor_msgs::PointCloud& cloud);
      void decayMap(char dec = 1);
      void Set(size_t i, size_t j, char val) {
        map[i][j] = val;
      }
      char Get(size_t i, size_t j) {
        return map[i][j];
      }
  };
}
