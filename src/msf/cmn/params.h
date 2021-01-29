#ifndef PARAMS_H_
#define PARAMS_H_


#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Header.h"

namespace msf {
namespace common {

    struct covariance
    {
        double attitude = 1.0;      // deg         
        double velocity = 0.5;      // m/s                    
        double position = 1.0;      // m              
        double gyrobias = 0.5;      // deg/s
        double accebias = 10;       // mg
    };

    struct sigma
    {
        double gyro = 0.05;         // deg/s
        double acce = 10;           // mg
    };

    struct noise
    {
        double gnss_yaw     = 0.2;      // deg
        double gnss_pos     = 0.2;      // m
        double lidar_yaw    = 0.2;      // deg
        double lidar_pos    = 0.2;      // m
        double odom_vel     = 0.2;      // m/s
    };

    struct limit
    {
        int yaw_max_cnt = 3000;
        int pos_max_cnt = 200;
        double yaw_chisqr = 50.0;
        double pos_chisqr = 50.0;
        double vel_chisqr = 20.0;
        double pos_error_switch = 0.6;
        double yaw_error_switch = 5;
        double pos_error_initial = 0.3;
        double yaw_error_initial = 2;
    };

    struct odom
    {
        double factor_l = 1.0;
        double factor_r = 1.0;
        double factor   = 1.0;
        double pitch_err= 0;
        double yaw_err  = 0;
    };
    
    struct gnss
    {
        int     zone        = 50;   //带号
        bool    hemi        = false;     
        double  base_x      = 443878.74;
        double  base_y      = 4436661.64;
        double  base_z      = 42.657;
        double  laram_x     = 0.0;
        double  laram_y     = 0.0;
        double  laram_z     = 0.0;
        double  baselen     = 1.0;
        double  baselen_dev = 0.2;
        double  angle       = 12.13;
    };

    struct imu
    {
        double abias_x  = 0.0;
        double abias_y  = 0.0;
        double abias_z  = 0.0;
        double gyro_var = 0.02;
        double acce_var = 0.01;
        double laram_x  = 0.0;
        double laram_y  = 0.0;
        double laram_z  = 0.0;
    };

    struct lidar
    {
        double ndt_stable = 2.5;
    };
    

    struct fusion
    {
        bool   debug        = false;
        double pubrate      = 50;    //hz
        double maxdrtime    = 10;  //s
    };
    

    struct params
    {
        covariance  covar_;
        sigma       sigma_;
        noise       noise_;
        limit       limit_;
        odom        odom_;
        imu         imu_;
        gnss        gnss_;
        lidar       lidar_;
        fusion      fusion_;
    };
}
}



#endif