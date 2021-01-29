#ifndef _CMN_H_
#define _CMN_H_

#include <math.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <utility>
#include <string>
#include <gtest/gtest.h>

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/SVD"

#include "params.h"


using namespace Eigen;

namespace msf {
namespace common {

    enum STATUS{
        NOFIX = 0,
        SINGLE,
        PSRDIFF,
        UNKNOW,
        NARROW_INT,
        NARROW_FLOAT,
        READY,
        GNSS,
        LIDAR,
        STATIC,
        FORWARD,
        BACKWARD,
        NORMAL,
        DEADROCKING,
        LOSS
    };

    const double kWGS84_RE = 6378137;    
    const double kWGS84_RA = 6378140;    
    const double kWGS84_RB = 6356755.3;  
    const double kWGS84_IF = 298.257223563;
    const double kWGS84_F = 0.003352810664747;  // WGS84_F = 1/WGS84_IF
    const double kOMEGA_IE = 7.2921151467e-5;

    const double kGRAVITY = 9.80665;                 // g
    const double kM_GRAVITY = kGRAVITY / 1000.0;     // mg
    const double kU_GRAVITY = kGRAVITY / 1000000.0;  // ug

    const double kPI = 3.141592653589793;  // pi
    const double kPI_2 = kPI / 2.0;        // pi/2
    const double kRAD2DEG = 57.295779513082323;  // rad -> deg
    const double kDEG2RAD = 0.017453292519943;   // deg -> rad

    #define PRECISION_D 1.0e-10
    #define ZEROS(x) (x < PRECISION_D && x > -PRECISION_D)
    #define SQ(x) (x*x)

    #define STATIC_S(x,y) (fabs(x) <= STATIC_VEL_THESHOLD && fabs(y) <= STATIC_VEL_THESHOLD)
    #define MOTION_S(x,y) (fabs(x) > MOTION_VEL_THESHOLD || fabs(y) > MOTION_VEL_THESHOLD)

    #define    SINL(lat) sin(lat * kDEG2RAD)
    #define    COSL(lat) cos(lat * kDEG2RAD)
    #define    TANL(lat) tan(lat * kDEG2RAD)
    #define   SIN2L(lat) sin(2.0 * lat * kDEG2RAD)

    #define      RM(lat) (kWGS84_RE * (1 - 2.0 * kWGS84_F + 3 * kWGS84_F * SQ(SINL(lat))))
    #define      RN(lat) (kWGS84_RE * (1 + kWGS84_F * SQ(SINL(lat))))

    #define G_LOCAL(lat) (kGRAVITY * (1 + 5.30244e-3 * SQ(SINL(lat)) - 5.82e-6 * SQ(SIN2L(lat))))

    typedef Eigen::Matrix<int, 7, 1> Vector7i;


    inline double Angle2Yaw(double input) {
        double output = 90 - input;
        if (output < -180) output += 360;
        if (output > 180) output -= 360;
        return output;
    }

    inline double Yaw2Angle(double input) {
        double output = 90 - input;
        if (output < -180) output += 360;
        if (output > 180) output -= 360;
        return output;
    }

    inline double LimitYaw(double input) {
        double output = input;
        if (output < -180) output += 360;
        if (output > 180) output -= 360;
        return output;
    }

    inline double Rms(double *buff, int len)
    {
        double accum = 0.0;
        if(len <= 0) return -1;
        for(int k = 0; k < len; k++)
        {
            accum += buff[k] * buff[k];
        }
        return sqrt(accum / len);
    }

    inline double Std(double *buff, int len)
    {
        double mean = 0.0;
        double accum = 0.0;
        if(len <= 1) return -1;
        for(int k = 0; k < len; k++)
        {
            mean += buff[k];
        }
        mean /= len;

        for(int k = 0; k < len; k++)
        {
            accum += (buff[k] - mean) * (buff[k] - mean);
        }
        return sqrt(accum / (len - 1));
    }

    inline Vector3d Cross(Vector3d src1, Vector3d src2)
    {
        Vector3d ret;
        ret(0) =  (src1(1) * src2(2) - src1(2) * src2(1));
        ret(1) = -(src1(0) * src2(2) - src1(2) * src2(0));
        ret(2) =  (src1(0) * src2(1) - src1(1) * src2(0));
        return ret;
    } 

    inline double Constrain(double val, double min, double max)
    {
        return (val < min) ? min : ((val > max) ? max : val);
    }
}
}









#endif