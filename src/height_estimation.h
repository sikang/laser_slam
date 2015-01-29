#include <iostream>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
#include <quadrotor_msgs/OutputData.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>
    
#define CONT_FLOOR_CNT_THR 60
#define FLOOR_THR          0.1
#define DIFF_FLOOR_THR     1.0
#define COMBINE_FLOOR_THR  0.6

namespace laser_slam
{
  class HeightEstimation
  {
    public:
      HeightEstimation();
      ~HeightEstimation();
      void set_laser_offset(double x, double y, double z);
      void process_height_measure(double height);
      void process_imu(const sensor_msgs::Imu& imu, const Eigen::Matrix3d& _Rp);
      double get_height(){return X(0);};
    private:
      // Current Floor Level
      double curr_floor;
      // Floor level to be published, for create multi-floor map in SLAM
      int cont_floor_cnt;
      double to_publish_floor;
      std::vector<double> to_publish_floor_hist;
      // Kalman Filter
      bool kalman_init_flag;
      bool floor_init_flag;
      // State 
      Eigen::Vector3d X;   
      // Covariance
      Eigen::Matrix<double,1,1> Q;    // Process noise, from IMU
      Eigen::Matrix<double,1,1> R;    // Measurement noise, from Laser
      Eigen::Matrix3d P;    // Overall covariance
      Eigen::Matrix3d Rp;    // Overall covariance

      double laser_offset_x;
      double laser_offset_y;
      double laser_offset_z;

  };

}

