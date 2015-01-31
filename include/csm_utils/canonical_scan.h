#ifndef CANONICAL_SCAN_H
#define CANONICAL_SCAN_H

#include <ros/ros.h>

#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min
#undef max
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <angles/angles.h>

namespace laser_slam
{

  class CanonicalScan
  {
    public:    
      CanonicalScan();
      void initParams(ros::NodeHandle &nh_private_);
      bool processScan2D(const LDP &curr_ldp_scan, const LDP &prev_ldp_scan, 
          const gtsam::Pose2 &initial_rel_pose, gtsam::Pose2& output_rel_pose,
          gtsam::Matrix& noise_matrix);
      bool pointCloudToLDP(const sensor_msgs::PointCloud &cloud, LDP &ldp);
      void laserScanToLDP(const sensor_msgs::LaserScan &scan_msg, LDP &ldp);
    protected:
      sm_params input_;
      sm_result output_;
      double csm_range_min_;
      double csm_range_max_;
      double min_theta;
  };

}

#endif
