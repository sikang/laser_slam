#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <gtsam_util.h>
#include "occ_grid.h"
#include "canonical_scan.h"
#include "scan_utils.hpp"

using namespace laser_slam;

ScanUtils scan_utils;
OccGrid occ_grid;
CanonicalScan scan_matcher;

static ros::Publisher prev_cloud_pub;
static ros::Publisher curr_cloud_pub;
static ros::Publisher covar_pub;
static ros::Publisher pose_pub;
static ros::Publisher map_pub;


static Eigen::Matrix3d R_;
static bool first_scan_;
static bool first_imu_;
static double prev_yaw_imu_;
static std_msgs::Header laser_header_;
static std_msgs::Header imu_header_;

static gtsam::Pose2 curr_pose_;
static gtsam::Pose2 predicted_pose_;

static LDP prev_ldp_;
static sensor_msgs::PointCloud prev_cloud_w_;

static int num_scan_ = 0;
static int decay_rate_ = 0;

void covarPub(const gtsam::Matrix& m)
{
  Eigen::Quaterniond q;
  Eigen::Vector3d v;

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

void update_map(const sensor_msgs::PointCloud& cloud_curr)
{
  sensor_msgs::PointCloud cloud_curr_w = 
    scan_utils.transform_cloud(cloud_curr, curr_pose_);
  if(num_scan_ > decay_rate_)
  {
    num_scan_ = 0;
    occ_grid.decayMap(1);
  }
  else
    num_scan_++;

  occ_grid.AddLaser(curr_pose_, scan_utils.down_sample_cloud(cloud_curr, 0.05));
  prev_cloud_w_ = scan_utils.merge_cloud(prev_cloud_w_, cloud_curr_w);
  prev_cloud_w_ = scan_utils.voxel_filter(prev_cloud_w_, 0.025);
  prev_cloud_w_ = occ_grid.filterCloud(prev_cloud_w_);

  sensor_msgs::PointCloud prev_cloud_b = 
    scan_utils.transform_cloud(prev_cloud_w_, curr_pose_.inverse());

  prev_cloud_b = scan_utils.pass_filter(prev_cloud_b);
  scan_utils.sort_cloud(prev_cloud_b);

  scan_matcher.pointCloudToLDP(prev_cloud_b, prev_ldp_);

  prev_cloud_w_ = scan_utils.transform_cloud(prev_cloud_b, curr_pose_);

  curr_cloud_pub.publish(cloud_curr_w);
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  laser_header_ = msg->header;
  sensor_msgs::LaserScan scan_out = scan_utils.scan_filter(*msg);
  sensor_msgs::PointCloud cloud = scan_utils.scan_to_cloud(scan_out, M_PI/4);
  sensor_msgs::PointCloud cloud2d = scan_utils.project_cloud(R_, cloud);
  sensor_msgs::PointCloud cloud_curr = scan_utils.down_sample_cloud(cloud2d, 0.025);

  LDP curr_ldp;
  scan_matcher.pointCloudToLDP(cloud_curr, curr_ldp);

  if(first_scan_)
  {
    first_scan_ = false;
    prev_cloud_w_ = cloud_curr;
    prev_ldp_ = curr_ldp;
    occ_grid.AddLaser(curr_pose_, scan_utils.down_sample_cloud(cloud_curr, 0.05));
  }
  else
  {
    prev_cloud_pub.publish(prev_cloud_w_);
    gtsam::Pose2 pose_diff;
    gtsam::Matrix noise_matrix;
    ros::Time t1 = ros::Time::now();
  
    bool valid_match = scan_matcher.processScan2D(curr_ldp, prev_ldp_, predicted_pose_, pose_diff, noise_matrix);
    if(!valid_match)
    {
      ROS_ERROR("cannot match!!!");
      return;
    }
    double dt = (ros::Time::now() - t1).toSec();
    if(dt > 0.01){
      ROS_INFO("Points [%zu, %zu]", prev_cloud_w_.points.size(), cloud_curr.points.size());
      ROS_INFO("Time cost for scan matcher: %f", dt);
    }
    t1 = ros::Time::now();

    curr_pose_ = curr_pose_.compose(pose_diff);
    update_map(cloud_curr);
    dt = (ros::Time::now() - t1).toSec();
    if(dt > 0.01)
      ROS_WARN("Total time cost for update map and scan: %f", dt);
    // Publishers 
    pose_pub.publish(transferPose2ToPoseStamped(curr_pose_, laser_header_));
    covarPub(noise_matrix);
  }
  predicted_pose_ = gtsam::Pose2();

  map_pub.publish(occ_grid.GetMap().map); 
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_header_ = msg->header;
  Eigen::Vector3d ypr = scan_utils.getYPRFromImu(msg);
  R_(0,0) = cos(ypr(1));
  R_(0,1) = sin(ypr(2))*sin(ypr(1));
  R_(0,2) = cos(ypr(2))*sin(ypr(1));
  R_(1,0) = 0;
  R_(1,1) = cos(ypr(2));
  R_(1,2) = -sin(ypr(2));
  R_(2,0) = -sin(ypr(1));
  R_(2,1) = sin(ypr(2))*cos(ypr(1));
  R_(2,2) = cos(ypr(2))*cos(ypr(1));

  if(!first_scan_)
  {
    if(!first_imu_){
      gtsam::Pose2 dp (0.0, 0.0, ypr(0) - prev_yaw_imu_);
      predicted_pose_ = predicted_pose_.compose(dp);
    }
    else
      first_imu_ = false;
    prev_yaw_imu_ = ypr(0);
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
  ROS_WARN("LaserSlam: decay_rate = %d", decay_rate_);


  scan_matcher.initParams(nh);

  R_ = Eigen::Matrix3d::Identity();
  first_scan_ = true;
  first_imu_ = true;
  curr_pose_ = gtsam::Pose2();

  ros::Subscriber scan_sub = nh.subscribe("scan_in", 10, scanCallback);
  ros::Subscriber imu_sub = nh.subscribe("imu_in", 10, imuCallback);
  ros::Subscriber outputdata_sub = nh.subscribe("output_data", 10, outputdataCallback);
  prev_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("prev_cloud", 10);
  curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("curr_cloud", 10);
  covar_pub = nh.advertise<visualization_msgs::Marker>("covar", 10);
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
  map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 5);


  ros::spin();

  return 0;
}


