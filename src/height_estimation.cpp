#include <csm_utils/height_estimation.h>
#include <stdio.h>

namespace laser_slam
{
  HeightEstimation::HeightEstimation()
  {
    curr_floor = 0.0;

    cont_floor_cnt = 0;                                                                    
    to_publish_floor = 0;                                                                  
    kalman_init_flag = false;
    floor_init_flag  = false;                                                              
    Q << 2.0 * 2.0;
    R << 0.02 * 0.02;
    P << 0.02, 0, 0,
        0, 0.02, 0,
        0, 0, 0.0001;
    X = Eigen::Vector3d::Zero();
    Rp = Eigen::Matrix3d::Identity();


    laser_offset_x = -0.02;
    laser_offset_y = 0.07;
    laser_offset_z = 0.273;
  } 

  
  HeightEstimation::~HeightEstimation()
  {
  }

  void HeightEstimation::set_laser_offset(double _x, double _y, double _z)
  {
    laser_offset_x = _x;
    laser_offset_y = _y;
    laser_offset_z = _z;
  }

  void HeightEstimation::process_height_measure(double height)
  {
    if (!kalman_init_flag)
      return; 

    //Transform height scan to horizontal frame
    Eigen::Vector3d h(laser_offset_x, laser_offset_y, -(height - laser_offset_z));
    Eigen::Vector3d hGlobal = Rp * h;
    double curr_laser_h = -hGlobal(2);
    // Handle floor levels
    if (!floor_init_flag)
    { 
      floor_init_flag = true;
      X(0) = curr_laser_h + curr_floor;
      to_publish_floor_hist.push_back(to_publish_floor);
    }
    if (fabs(X(0) - (curr_laser_h + curr_floor)) >= FLOOR_THR)
    { 
      //ROS_WARN(" ----- Floor Level Changed ----- ");
      curr_floor = X(0) - curr_laser_h;
      cont_floor_cnt = 0;  
    }   
    else
    {   
      cont_floor_cnt++; 
      if (cont_floor_cnt == CONT_FLOOR_CNT_THR && fabs(curr_floor-to_publish_floor) > DIFF_FLOOR_THR)
      {   
        // Try combine floor levels
        double min_dist = 100;
        double min_idx  = 0;
        for (unsigned int k = 0; k < to_publish_floor_hist.size(); k++)
        {
          double dist = fabs(to_publish_floor_hist[k] - curr_floor);
          if (dist < min_dist)
          {
            min_dist = dist;
            min_idx = k;
          }
        }
        if (min_dist < COMBINE_FLOOR_THR)
        { 
          to_publish_floor = to_publish_floor_hist[min_idx];
          curr_floor = to_publish_floor;
          X(0) = curr_laser_h + curr_floor;
        }
        else
        {
          to_publish_floor = curr_floor;
          to_publish_floor_hist.push_back(to_publish_floor);
        } 
      }
    }
    X(2) = curr_floor;
    Eigen::MatrixXd C(1,3);
    C(0,0) = 1; 
    C(0,1) = 0; 
    C(0,2) = -1;
    Eigen::Vector3d K = P*C.transpose() * (C*P*C.transpose() + R).inverse();
    Eigen::Matrix<double, 1, 1> curr_laser_H;
    curr_laser_H << curr_laser_h;

    X = X + K * (curr_laser_H - C*X);
    P = P - K*C*P;
    curr_floor = X(2); 
  }

  void HeightEstimation::process_imu(const sensor_msgs::Imu& imu, const Eigen::Matrix3d& _Rp)
  {
    Rp = _Rp;
    // Get acceleration
    Eigen::Vector3d a (imu.linear_acceleration.x,
        imu.linear_acceleration.y,
        imu.linear_acceleration.z);
    // Calibration, estimate gravity
    static int calLimit = 100;
    static int calCnt   = 0;
    static Eigen::Vector3d g = Eigen::Vector3d::Zero();
    if (calCnt < calLimit)
    {      
      calCnt++;
      g += a;
      return;
    }      
    else if (calCnt == calLimit)
    {      
      calCnt++;
      g /= calLimit;
      g(0) = 0;
      g(1) = 0;
      printf("g = %f !!!\n\n", g(2));
      return;
    }      
    // Init process update time
    static ros::Time prev_update;
    static ros::Time curr_update;
    if (!kalman_init_flag)
    {      
      kalman_init_flag = true;
      prev_update = imu.header.stamp;
      curr_update = imu.header.stamp;
      return;
    }      
    // Transform acceleration to global frame
    Eigen::Vector3d ag = Rp * a - g;  // Substract gravity force
    double az = ag(2);
    // Get update time interval
    curr_update = imu.header.stamp;
    double dt = (curr_update - prev_update).toSec();
    prev_update = curr_update;
    // Process Update
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0,1) = dt;
    Eigen::MatrixXd B(3,1);
    B(0,0) = dt*dt/2;
    B(1,0) = dt;
    B(2,0) = 0;
    X = A*X + B*az;
    P = A*P*A.transpose() + B*Q*B.transpose();
  }
}
