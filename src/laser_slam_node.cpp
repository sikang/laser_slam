#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <gtsam_util.h>
#include "occ_grid.h"
#include "canonical_scan.h"
#include "scan_utils.hpp"
#include "height_estimation.h"

using namespace laser_slam;

ScanUtils scan_utils;
OccGrid occ_grid;
CanonicalScan scan_matcher;
HeightEstimation height_estimation;

static ros::Publisher laser_cloud_pub;
static ros::Publisher prev_cloud_pub;
static ros::Publisher curr_cloud_pub;
//static ros::Publisher covar_pub;
//static ros::Publisher pose_pub;
static ros::Publisher odom_pub;
static ros::Publisher map_pub;

static Eigen::Matrix3d R_;
static bool first_scan_;
static bool first_imu_;
static double prev_yaw_imu_;
static std_msgs::Header laser_header_;

static gtsam::Pose2 curr_pose_;
static gtsam::Pose2 predicted_pose_;

static LDP prev_ldp_;
static sensor_msgs::PointCloud prev_cloud_w_;
static Eigen::Vector3d ypr_ = Eigen::Vector3d::Zero();

static int num_scan_ = 0;
static int decay_rate_;
static std::string world_frame;
static double occ_res_;
static double cloud_res_;
static double shift_yaw_;

/*
void covarPub(const gtsam::Matrix& m, Eigen::Vector3d& v)
{
  Eigen::Quaterniond q;
  bool valid = eigenValue(m, q, v);

  if(valid){
    visualization_msgs::Marker covar;
    covar.ns = "covar_visualization";
    covar.header = laser_header_;
    covar.id = 0;
    covar.type = visualization_msgs::Marker::SPHERE;
    covar.action = visualization_msgs::Marker::ADD;
    covar.pose.position.x = 0;
    covar.pose.position.y = 0;
    covar.pose.position.z = 0;
    covar.pose.orientation.w = q.w();
    covar.pose.orientation.x = q.x();
    covar.pose.orientation.y = q.y();
    covar.pose.orientation.z = q.z();
    int scale = 1000000;
    covar.scale.x = v[0]*scale * 10;
    covar.scale.y = v[1]*scale;
    covar.scale.z = v[2]*scale;
    covar.color.a = 0.5;
    covar.color.r = 0.5;
    covar.color.g = 0;
    covar.color.b = 0.5;
    covar_pub.publish(covar);
  }
}
*/


void publishOdom(const geometry_msgs::PoseStamped& pose)
{
  nav_msgs::Odometry odom;
  odom.header = pose.header;
  odom.pose.pose = pose.pose;
  odom.pose.covariance[0+0*6] = 0.01*0.01;
  odom.pose.covariance[1+1*6] = 0.01*0.01;
  odom.pose.covariance[2+2*6] = 0.01*0.01;
  odom.pose.covariance[3+3*6] = 0.01*0.01;
  odom.pose.covariance[4+4*6] = 0.03*0.03;
  odom.pose.covariance[5+5*6] = 0.03*0.03;

  odom_pub.publish(odom);
}

void update_map(const sensor_msgs::PointCloud& cloud_curr)
{
  sensor_msgs::PointCloud cloud_curr_w = 
    scan_utils.transform_cloud(cloud_curr, curr_pose_, world_frame);

  if(num_scan_ > decay_rate_)
  {
    num_scan_ = 0;
    occ_grid.decayMap(1);
  }
  else
    num_scan_++;

  occ_grid.AddLaser(curr_pose_, scan_utils.down_sample_cloud(cloud_curr, occ_res_));

  prev_cloud_w_ = scan_utils.merge_cloud(prev_cloud_w_, cloud_curr_w);
  prev_cloud_w_ = scan_utils.voxel_filter(prev_cloud_w_, cloud_res_);


  prev_cloud_w_ = occ_grid.filterCloud(prev_cloud_w_);

  sensor_msgs::PointCloud prev_cloud_b = 
    scan_utils.transform_cloud(prev_cloud_w_, curr_pose_.inverse(), cloud_curr.header.frame_id);

  prev_cloud_b = scan_utils.pass_filter(prev_cloud_b);

  scan_matcher.pointCloudToLDP(prev_cloud_b, prev_ldp_);

  prev_cloud_w_ = scan_utils.transform_cloud(prev_cloud_b, curr_pose_, world_frame);

  curr_cloud_pub.publish(cloud_curr_w);
}



void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ros::Time t0 = ros::Time::now();

  laser_header_ = msg->header;

  double laser_height, laser_height_cov;

  sensor_msgs::LaserScan scan_out = scan_utils.scan_filter(*msg, laser_height, laser_height_cov);
  if(laser_height_cov > 0.1)
    ROS_WARN_THROTTLE(1, "Laser height = %f, cov = %f", laser_height, laser_height_cov);

  height_estimation.process_height_measure(laser_height);

  sensor_msgs::PointCloud cloud = scan_utils.scan_to_cloud(scan_out, shift_yaw_);
  cloud.header = laser_header_;
  laser_cloud_pub.publish(cloud);
  sensor_msgs::PointCloud cloud2d = scan_utils.project_cloud(R_, cloud);
  sensor_msgs::PointCloud cloud_curr = scan_utils.down_sample_cloud(cloud2d, cloud_res_);

  LDP curr_ldp;
  scan_matcher.pointCloudToLDP(cloud_curr, curr_ldp);


  if(first_scan_)
  {
    first_scan_ = false;
    prev_cloud_w_ = cloud_curr;
    prev_ldp_ = curr_ldp;
    occ_grid.AddLaser(curr_pose_, scan_utils.down_sample_cloud(cloud_curr, occ_res_));
  }
  else
  {
    prev_cloud_pub.publish(prev_cloud_w_);
    gtsam::Pose2 pose_diff;
    gtsam::Matrix noise_matrix;

    ros::Time t1 = ros::Time::now();
    scan_matcher.processScan2D(curr_ldp, prev_ldp_, predicted_pose_, pose_diff, noise_matrix);
    curr_pose_ = curr_pose_.compose(pose_diff);
    gtsam::Pose3 curr_pose3 = transferPose2ToPose3(curr_pose_, height_estimation.get_height(), ypr_(2), ypr_(1));
    geometry_msgs::PoseStamped pose_stamped = transferPoseToPoseStamped(curr_pose3, laser_header_);
    //covarPub(noise_matrix);
    //pose_pub.publish(pose_stamped);
    publishOdom(pose_stamped);
    double dt = (ros::Time::now() - t1).toSec();
    if(dt > 0.01)
      ROS_WARN("Time cost for process scan matcher: %f", dt);
   // else
   //   ROS_INFO("Time cost for process scan matcher: %f", dt);

    update_map(cloud_curr);
  }
  predicted_pose_ = gtsam::Pose2();

  double dt = (ros::Time::now() - t0).toSec();
  if(dt > 0.020){
    ROS_WARN("Points [%zu, %zu]", prev_cloud_w_.points.size(), cloud_curr.points.size());
    ROS_WARN("Time cost for process one scan: %f", dt);
  }
  map_pub.publish(occ_grid.GetMap(world_frame).map); 
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ypr_ = scan_utils.getYPRFromImu(msg);
  R_(0,0) = cos(ypr_(1));
  R_(0,1) = sin(ypr_(2))*sin(ypr_(1));
  R_(0,2) = cos(ypr_(2))*sin(ypr_(1));
  R_(1,0) = 0;
  R_(1,1) = cos(ypr_(2));
  R_(1,2) = -sin(ypr_(2));
  R_(2,0) = -sin(ypr_(1));
  R_(2,1) = sin(ypr_(2))*cos(ypr_(1));
  R_(2,2) = cos(ypr_(2))*cos(ypr_(1));

  height_estimation.process_imu(*msg, R_);

  if(!first_scan_)
  {
    if(!first_imu_){
      gtsam::Pose2 dp (0.0, 0.0, ypr_(0) - prev_yaw_imu_);
      predicted_pose_ = predicted_pose_.compose(dp);
    }
    else
      first_imu_ = false;
    prev_yaw_imu_ = ypr_(0);
  } 
}

void outputdataCallback(const quadrotor_msgs::OutputData::ConstPtr& msg)
{
  sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);
  imu_msg->header = msg->header;
  imu_msg->orientation = msg->orientation;
  imu_msg->angular_velocity = msg->angular_velocity;
  imu_msg->linear_acceleration = msg->linear_acceleration;

  imuCallback(imu_msg);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "laser_slam_node");

  ros::NodeHandle nh("~");
 
  nh.param("decay_rate", decay_rate_, -1);
  nh.param("world_frame", world_frame, std::string("map"));
  ROS_WARN("LaserSlam: decay_rate = %d", decay_rate_);


  // Scan filter params
  int idx_width, idx_middle, height_idx_low, height_idx_up, min_ang_idx;
  double min_theta, range_theta;
  nh.param("idx_width", idx_width, 38);
  nh.param("idx_middle", idx_middle, 968);
  nh.param("height_idx_low", height_idx_low, 0);
  nh.param("height_idx_up", height_idx_up, 10);
  nh.param("min_ang_idx", min_ang_idx, 0);
  nh.param("min_theta", min_theta, -M_PI/2);
  nh.param("range_theta", range_theta, M_PI*7/4);

  // Height estimation params
  double laser_offset_x, laser_offset_y, laser_offset_z;
  nh.param("laser_offset_x", laser_offset_x, -0.02);
  nh.param("laser_offset_y", laser_offset_y, 0.07);
  nh.param("laser_offset_z", laser_offset_z, 0.273);

  nh.param("shift_yaw", shift_yaw_, M_PI/4);

  nh.param("occ_res", occ_res_, 0.1);
  nh.param("cloud_res", cloud_res_, 0.08);

  scan_matcher.initParams(nh);
  occ_grid.SetRes(occ_res_);
  scan_utils.init_scan_filter(idx_width, idx_middle,
      height_idx_low, height_idx_up,
      min_theta, range_theta,
      min_ang_idx);
  height_estimation.set_laser_offset(laser_offset_x,
      laser_offset_y, laser_offset_z);
  ROS_WARN("min_ang_idx: %d", min_ang_idx);

  R_ = Eigen::Matrix3d::Identity();
  first_scan_ = true;
  first_imu_ = true;
  curr_pose_ = gtsam::Pose2();

  ros::Subscriber scan_sub = nh.subscribe("scan_in", 10, scanCallback);
  ros::Subscriber imu_sub = nh.subscribe("imu_in", 10, imuCallback);
  ros::Subscriber outputdata_sub = nh.subscribe("output_data", 10, outputdataCallback);
  laser_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("laser_cloud", 10);

  prev_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("prev_cloud", 10);
  curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("curr_cloud", 10);

  // Disable covariance from csm
  // covar_pub = nh.advertise<visualization_msgs::Marker>("covar", 10);
  // Disable pose pub
  // pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 5);


  ros::spin();

  return 0;
}


