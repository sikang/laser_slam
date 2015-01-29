#include "scan_utils.hpp"
#include <stdio.h>

namespace laser_slam
{

  bool comp(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
  {
    return (p1.z < p2.z);
  }

  Eigen::Vector3d ScanUtils::getYPRFromImu(const sensor_msgs::Imu::ConstPtr& msg)
  {
    double q0 = msg->orientation.w;
    double q1 = msg->orientation.x;
    double q2 = msg->orientation.y;
    double q3 = msg->orientation.z;

    Eigen::Vector3d ypr;
    ypr(0) = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
    ypr(1) = asin(2*(q0*q2 - q3*q1));
    ypr(2) = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    return ypr;
  }

  sensor_msgs::PointCloud ScanUtils::transform_cloud(const sensor_msgs::PointCloud& cloud, const gtsam::Pose2& pose, const std::string& frame_id)
  {
    sensor_msgs::PointCloud new_cloud = cloud;
    new_cloud.header.frame_id = frame_id;
    double co = cos(pose.theta());
    double so = sin(pose.theta());
    for(size_t i = 0; i < new_cloud.points.size(); i++)
    {              
      new_cloud.points[i].x = pose.x() + 
        co * cloud.points[i].x - 
        so * cloud.points[i].y;
      new_cloud.points[i].y = pose.y() +
        so * cloud.points[i].x +
        co * cloud.points[i].y;
    }     
    return new_cloud;    
  }

  sensor_msgs::PointCloud ScanUtils::merge_cloud(const sensor_msgs::PointCloud& cloud1, const sensor_msgs::PointCloud& cloud2)
  {
    sensor_msgs::PointCloud new_cloud = cloud1;
    new_cloud.header = cloud2.header;
    new_cloud.points.insert(new_cloud.points.end(), cloud2.points.begin(), cloud2.points.end());
    return new_cloud;
  }

  sensor_msgs::PointCloud ScanUtils::pass_filter(const sensor_msgs::PointCloud& cloud)
  {
    sensor_msgs::PointCloud new_cloud;
    new_cloud.header = cloud.header;

    for(size_t i = 0; i< cloud.points.size(); i++ )
    {
      geometry_msgs::Point32 pt = cloud.points[i];
      double d = hypot(pt.x, pt.y);
      pt.z = angles::normalize_angle_positive(atan2(pt.y, pt.x) - min_theta);
      if(d >= 0.1 && d <= 30 && pt.z < range_theta)
        new_cloud.points.push_back(pt);
    }
    std::sort(new_cloud.points.begin(), new_cloud.points.end(), comp);

    for(size_t i = 0; i< new_cloud.points.size(); i++)
      new_cloud.points[i].z = 0;
   return new_cloud;
  }



  sensor_msgs::PointCloud ScanUtils::down_sample_cloud(const sensor_msgs::PointCloud& cloud, double res)
  { 
   // Only works for point cloud directly from laser scan 
    sensor_msgs::PointCloud new_cloud;
    new_cloud.header = cloud.header;
    Eigen::Vector3d prev (100, 100, 100);
    for (size_t k = 0; k < cloud.points.size(); k++)
    {
      Eigen::Vector3d vec(cloud.points[k].x,
          cloud.points[k].y,
          cloud.points[k].z);
      if ( (vec-prev).norm() >= res)
      {
        geometry_msgs::Point32 p;
        p.x = vec[0];
        p.y = vec[1];
        p.z = vec[2];
        new_cloud.points.push_back(p);
        prev = vec;
      }
    }
    return new_cloud;
  }  

  sensor_msgs::PointCloud ScanUtils::voxel_filter(const sensor_msgs::PointCloud& cloud, double res)
  {
    sensor_msgs::PointCloud new_cloud;
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
    pcl::PCLPointCloud2::Ptr pcl_cloud2(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(cloud2, *pcl_cloud2);

    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (pcl_cloud2);
    sor.setLeafSize (res, res, res);
    sor.filter (*cloud_filtered);

    sensor_msgs::PointCloud2 cloud_output;
    pcl_conversions::fromPCL(*cloud_filtered, cloud_output);
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_output, new_cloud);

    return new_cloud;
  }


  sensor_msgs::PointCloud ScanUtils::scan_to_cloud(const sensor_msgs::LaserScan& scan, double theta)
  {
    sensor_msgs::PointCloud::Ptr cloud_ptr(new sensor_msgs::PointCloud);
    cloud_ptr->header = scan.header;
    float angle_min = scan.angle_min + theta;
    float angle_increment = scan.angle_increment;
    for(size_t i = min_ang_idx; i < scan.ranges.size() - min_ang_idx; i++)
    {
      if(0.1 <= scan.ranges[i] && scan.ranges[i] <= scan.range_max){
        float curr_angle = angle_min + i * angle_increment;
        float x = scan.ranges[i]*cos(curr_angle);
        float y = scan.ranges[i]*sin(curr_angle);

        geometry_msgs::Point32 point;
        point.x = x;
        point.y = y;
        point.z = 0;
        cloud_ptr->points.push_back(point);
      }
    }
    return *cloud_ptr;
  }

  sensor_msgs::PointCloud ScanUtils::project_cloud(const Eigen::Matrix3d& R, const sensor_msgs::PointCloud& cloud)
  {
    sensor_msgs::PointCloud new_cloud;
    new_cloud.header = cloud.header;

    for (size_t k = 0; k < cloud.points.size(); k++)
    {
      Eigen::Vector3d pl(cloud.points[k].x, cloud.points[k].y, cloud.points[k].z);
      Eigen::Vector3d pw = R * pl;
      geometry_msgs::Point32 p;
      p.x = pw(0);
      p.y = pw(1);
      p.z = 0;
      new_cloud.points.push_back(p);
    }
    return new_cloud;
  }



  void ScanUtils::init_scan_filter(int _idx_width, int _idx_middle, 
      int _height_idx_low, int _height_idx_up,
      double _min_theta, double _range_theta,
      int _min_ang_idx)
  {
    idx_width = _idx_width;
    idx_middle = _idx_middle;
    height_idx_low = _height_idx_low;
    height_idx_up = _height_idx_up;

    min_theta = _min_theta;
    range_theta = _range_theta;

    min_ang_idx = _min_ang_idx;
  }



  sensor_msgs::LaserScan ScanUtils::scan_filter(const sensor_msgs::LaserScan& scan, double &median, double &stdev)
  {
    std::vector<double> heights;

    sensor_msgs::LaserScan scan_filtered = scan;
    for(int k = -idx_width; k < idx_width; k++){
      if(k >= height_idx_low && k <= height_idx_up)
        heights.push_back(scan_filtered.ranges[idx_middle + k]);
      scan_filtered.ranges[idx_middle + k] = std::numeric_limits<float>::quiet_NaN();
    }


    std::sort (heights.begin(), heights.end());
    median = heights[heights.size() / 2];

    double sum = std::accumulate(heights.begin(), heights.end(), 0.0);
    double mean = sum / heights.size();

    double sq_sum = std::inner_product(heights.begin(), heights.end(), heights.begin(), 0.0);
    stdev = std::sqrt(sq_sum / heights.size() - mean * mean);

    return scan_filtered;
  }

}
