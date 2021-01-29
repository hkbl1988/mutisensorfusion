
#ifndef _MUTI_FUSION_H
#define _MUTI_FUSION_H
#include <gtest/gtest.h>
#include <math.h>


#include "vardef.h"
#include "quaternion.h"
#include "ekf.h"
#include "preprocess.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <random>

//test add 
#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>

using namespace Eigen;
using namespace std;

namespace msf {
namespace common {


    class mutifusion {
    public:
      /**
      * @brief Constructor which defers initialization mutifusion
      */
      mutifusion();
      /**
       * @brief Destructor
       */
      ~mutifusion();


      bool SetImuPose(const ImuPose &msg);

      bool SetOdomPose(const OdomPose &msg);

      bool SetGnssPose(const GnssPose &msg);

      bool SetLidarPose(const LidarPose &msg);

      void ResetFusionMode(STATUS mode);

      void ApplyNav();
      
      output_elements GetFusionResult();

      void ResetFilter();


    private:

      void initialization();

      void initial_param();

      void initial_system();

      void initial_pvy(Vector3d pos,double yaw);

      void set_outdata_elements(output_elements &outmsg,imu_elements &imumsg);

      void quaternion_update(Vector3d &omega_i_b,const Vector3d vel,const double tau);
      void position_velocity_update(Vector3d &pos, Vector3d &vel,const Vector3d fibb, const double tau);
      void inertial_error_model(const Vector3d fibb, const Vector3d vel, const double tau);

      double apply_yaw_fusion(double yaw_ref, double noise, size_t index);
      Vector3d apply_vel_fusion(Vector3d vel_ref, Matrix3d cbnV,double noise, size_t index);
      Vector3d apply_vel_fusion(Vector3d vel_ref, double noise, size_t index);
      Vector3d apply_pos_fusion(Vector3d pos_ref, double noise, size_t index);

      Vector3d apply_posoffset_estimation(Vector3d pos_ref, double noise, size_t index);
      double apply_yawoffset_estimation(double yaw_ref, double noise, size_t index);

      void apply_feedback(feedback_elements &msg);

      void apply_deltback(dpos_elements &msg);
      void apply_deltback(dang_elements &msg);

      void switch_out_data();
      void set_measure_noise(); 

      Vector3d get_delt_position(Vector3d pos,size_t index);
      double get_delt_yaw(double yaw,size_t index);
      void get_estimate_states();
      
      void quat_correct(Vector3d attError);  

      void cache_output_data(const output_elements &msg);
      void cache_fusion_data(const output_elements &msg);
      void cache_gnss_data(const gnss_pos_elements &msg);
      void cache_ins_data(const output_elements &msg);
      void cache_imu_data(const imu_elements &msg);

      size_t get_suitable_index(const double timetic);
      double linear_interp_yaw(double src1, double src2, double coeff);
      double linear_interp(double src1, double src2, double coeff);
      double delt_yaw(const double yaw1 , const double yaw2);
      void limit_yaw(double &yaw);

      void reset_sys_position(feedback_elements &fd,const Vector3d pos,const size_t index);
      void reset_sys_yaw(feedback_elements &fd,const double yaw,const size_t index);

      output_elements smooth_fusion_output();

      double recognize_gnss_error(gnss_pos_elements pos_msg, double timetic);

      void check_gnss_quality(gnss_pos_elements pos_msg, gnss_yaw_elements yaw_msg);
      void check_lidar_quality(lidar_elements lidar_msg);
      void check_localization_status();
      void check_observation_update();
      void check_odom_quality();
      STATUS check_vehicle_status(double angular_l,double angular_r);

      void reset_dr_position();
      void rockoning_time_proc();
      void out_velocity_limit();
      void out_inertial_fetch();
      void handling_exception(Matrix3d cbn);

      void save_prints();

    private:

      params params_; 

      
      double latitude_ = 40;
      const int map_region_path_len_ = 17;
      bool get_map_region_path_flag_ = false;

      ofstream nav_out_;
      ofstream cov_out_;

      KalmanFilter<double, ADIM, RDIM, UDIM> ekf_;

      Quaternion quaternion_;

      Matrix<double, ADIM,    1> States_;
      Matrix<double, ADIM, ADIM> P_;
      Matrix<double, ADIM, ADIM> A_;
      Matrix<double, ADIM, ADIM> F_;
      Matrix<double, ADIM, GDIM> G_;
      Matrix<double, GDIM, GDIM> Q_;
      Matrix<double, ADIM, ADIM> Qk_;
      Matrix<double,1,ADIM> H_;
      Matrix<double,1,1> R_;
      Matrix<double,1,1> Z_;

      Matrix<double, ADIM, ADIM> Pinit_;


      double yawInnov_ = 0.0;
      double yawoffsetInnov_ = 0.0;
      Vector3d velInnov_;
      Vector3d velInnov_gnss_;
      Vector3d posInnov_;
      Vector3d posoffsetInnov_;

      imu_elements            imu_data_;
      odom_elements           odom_data_;
      gnss_pos_elements       gnss_pos_data_;
      gnss_yaw_elements       gnss_yaw_data_;     
      lidar_elements          lidar_data_;
      output_elements         out_data_;

      output_elements   out_fusion_;

      feedback_elements    odom_feedback_;
      feedback_elements   lidar_feedback_;
      feedback_elements    gnss_feedback_;

      dpos_elements     dpos_data_;
      dpos_elements     dpos_odom_data_;
      dang_elements     dang_data_; 

      average_elements imu_avge_;
      average_elements gnss_avge_;

      covariance_elements  ekf_covariance_;

      imu_variance_elements imu_variance_;

      //out data deque
      deque<output_elements> out_data_deque_;
      size_t out_deque_max_size_;
      mutex out_data_deque_mutex_;

      //ekf fusion mutex
      mutex ekf_fusion_mutex_;

      //odom data deque
      deque<output_elements> fusion_data_deque_;
      size_t fusion_deque_max_size_;
      mutex fusion_data_deque_mutex_;

      //gnss data deque
      deque<gnss_pos_elements> gnss_data_deque_;
      size_t gnss_deque_max_size_;

      //ins data deque
      deque<output_elements> ins_data_deque_;
      size_t ins_deque_max_size_;

      //imu data deque
      deque<imu_elements> imu_data_deque_;
      size_t imu_deque_max_size_;

      Vector3d att_install_error_;

      atomic<bool> system_initial_flag_;
      atomic<bool> yaw_initial_flag_;
      atomic<bool> horizontal_initial_flag_;
      atomic<bool> position_initial_flag_;
      atomic<bool> velocity_initial_flag_;
      atomic<bool> gyroscope_initial_flag_;
      atomic<bool> pos_yaw_initial_flag_;
      atomic<bool> check_error_flag_;

      atomic<bool> gnss_error_over_limit_flag_;
      atomic<bool> lidar_error_over_limit_flag_;

      unsigned int imu_time_ms_psd_ = 0;
      double imu_time_ros_psd_ = 0.0;
      double odom_time_ros_psd_ = 0.0;
      double gnss_time_ros_psd_ = 0.0;
      double lidar_time_ros_psd_ = 0.0;

      unsigned int clk_system_ = 0;

      double gnss_yaw_noise_ = 0.5;
      double gnss_pos_noise_ = 0.2;
      double gnss_vel_noise_ = 0.1;
      double lidar_yaw_noise_ = 0.2;
      double lidar_pos_noise_ = 0.2;
      double odom_vel_noise_ = 0.2; 

      double lambda_yaw_ = 0.0;
      double lambda_yaw_offset_ = 0.0;
      double lambda_pos_[3] = {0.0};
      double lambda_pos_offset_[3] = {0.0};
      double lambda_vel_[3] = {0.0};
      double lambda_vel_gnss_[3] = {0.0};

      double odom_velocity_l_ = 5.0;
      double odom_velocity_r_ = 5.0;

      Vector3d delt_pos_ref_;
      Vector3d delt_yaw_ref_;

      Vector3d delt_odom_pos_;

      STATUS map_region_cur_ = STATUS::UNKNOW;

      STATUS fusion_type_ = STATUS::LIDAR;
      STATUS set_type_ = STATUS::READY;

      unsigned int  rcv_odom_cnt_ = 0;
      unsigned int  rcv_gnss_cnt_ = 0;
      unsigned int rcv_lidar_cnt_ = 0;

      unsigned int reset_localization_lidar_ = 0;
      unsigned int reset_localization_gnss_ = 0;

      unsigned int rcv_gnss_yaw_valid_cnt_ = 0;
      unsigned int rcv_gnss_pos_valid_cnt_ = 0;

      unsigned int gnss_yaw_pos_valid_cnt_ = 0;

      unsigned int rcv_gnss_cnt_tal_ = 0;

      unsigned int recog_gnss_cnt_ = 0;

      int static_cnt_ = 0;
      int vehicle_forward_cnt_ = 0;
      int vehicle_backward_cnt_ = 0;
      STATUS vehicle_status_ = STATUS::UNKNOW;

      bool is_debug_ = false;
      bool is_yaw_filter_ = false;
      bool is_yawoffset_filter_ = false;
      bool is_pos_filter_[3] = {false};
      bool is_posoffset_filter_[3] = {false};
      bool is_odom_vel_filter_[3] = {false};
      bool is_gnss_vel_filter_[3] = {false};

      Vector3d gnss_pos_dev_;
      double gnss_yaw_dev_ = 0.0;

      Vector3d lidar_pos_dev_;
      double lidar_yaw_dev_ = 0.0;

      int gnss_valid_cnt_ = 0;
      int lidar_valid_cnt_ = 0;   

      int system_initial_times_ = 0;

      Vector3d initial_pos_error_;
      double  initial_yaw_error_ = 0;

      int system_initial_wait_cnt_ = 0;

      int pos_error_rms_cnt_ = 0;
      bool pos_error_rms_flag_ = false;
      double pos_error_rms_ = 0.0;
      int collision_cnt = 0;
      Vector3d imu_laram_;
    };



  }// namespace localization
}// namespace avos

#endif
