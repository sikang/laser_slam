#include "canonical_scan.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/OutputData.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


namespace laser_slam
{
  class ScanUtils
  {
    public:
      Eigen::Vector3d getYPRFromImu(const sensor_msgs::Imu::ConstPtr& msg);
      sensor_msgs::PointCloud transform_cloud(const sensor_msgs::PointCloud& cloud, const gtsam::Pose2& pose, const std::string& frame_id);
      sensor_msgs::PointCloud merge_cloud(const sensor_msgs::PointCloud& cloud1, const sensor_msgs::PointCloud& cloud2);
      sensor_msgs::PointCloud pass_filter(const sensor_msgs::PointCloud& cloud);
      sensor_msgs::PointCloud project_cloud(const Eigen::Matrix3d& R, const sensor_msgs::PointCloud& cloud);
      sensor_msgs::PointCloud down_sample_cloud(const sensor_msgs::PointCloud& cloud, double res);
      sensor_msgs::PointCloud voxel_filter(const sensor_msgs::PointCloud& cloud, double res);
      void sort_cloud(sensor_msgs::PointCloud& cloud);
      sensor_msgs::LaserScan scan_filter(const sensor_msgs::LaserScan& scan, std::vector<double>& heights);
      sensor_msgs::PointCloud scan_to_cloud(const sensor_msgs::LaserScan& scan, double theta = 0.0);
      void init_scan_filter(int _idx_width, int _idx_middle, 
          int _height_idx_low, int _height_idx_up,
          double _min_theta, double _range_theta);

    private:
      int idx_width;
      int idx_middle;
      int height_idx_low;
      int height_idx_up;

      int min_theta;
      int range_theta;
  };

}

