#ifndef _PRE_PROCESS_H
#define _PRE_PROCESS_H

#include "sensor/imu.h"
#include "sensor/gnss.h"
#include "sensor/odom.h"
#include "sensor/lidar.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include "utm.h"
#include "cmn.h"

using namespace Eigen;


#define PI 3.141592654
#define TODEG (180 / 3.141592654)
#define TORAD (3.141592654 / 180)
#define NDT_FREQ 3


namespace msf {
namespace common {

    struct LidarPose 
    {
        std_msgs::Header header;
        double xg,yg,zg;
        double roll,pitch,yaw;
        double trans_probability;
        void GetMsg(const sensor::lidar &msg) 
        {
            header.stamp = msg.header.stamp;
            trans_probability = msg.trans_probability;
            xg = msg.ndt_pose.pose.position.x;
            yg = msg.ndt_pose.pose.position.y;
            zg = msg.ndt_pose.pose.position.z;
            double tf_roll, tf_pitch, tf_angle;
            tf::Quaternion tf_quaternion;
            quaternionMsgToTF(msg.ndt_pose.pose.orientation, tf_quaternion);
            tf::Matrix3x3(tf_quaternion).getRPY(tf_roll, tf_pitch, tf_angle);
            yaw = tf_angle * TODEG;
        }
    };


    struct GnssPose 
    {
        std_msgs::Header header;
        double utc;
        long zone;
        char hemisphere;
        int satenum;
        int status,yawstatus;
        int yawvalid;
        double xg,  yg,  zg;
        double lat, lon, alt;
        double yaw, yawstd;
        double vel;
        double trk;
        double hdop, age;
        double baselen;
        void GetMsg(const sensor::gnss &msg)
        {
            header.stamp = msg.header.stamp;
            utc = msg.utc;
            satenum = msg.satenum;
            status = msg.status;
            yawstatus = msg.yawstatus;
            yawvalid = msg.yawvalid;
            //需要加入经纬度转UTM函数
            lat = msg.lat;
            lon = msg.lon;
            Convert_Geodetic_To_UTM(lat*TORAD,lon*TORAD,&zone,&hemisphere,&xg,&yg);
            alt = zg = msg.alt;
            yaw = msg.yaw;
            yawstd = msg.yawstd;
            vel = msg.vel;
            trk = msg.trk;
            hdop = msg.hdop;
            age = msg.age;
            baselen = msg.baselen;
        }
    };

    struct OdomPose 
    {
        std_msgs::Header header;
        int dFrameCnt;
        double velocity_rl;
        double velocity_rr;
        double velocity;
        void GetMsg(const sensor::odom &msg) 
        {
            header = msg.header;
            dFrameCnt = msg.dFrameCnt;
            velocity_rl = msg.velocity_rl;
            velocity_rr = msg.velocity_rr;
            velocity = msg.velocity;
            
        }
    };

    struct ImuPose 
    {
        std_msgs::Header header;
        unsigned int tag;
        double gyro_x;
        double gyro_y;
        double gyro_z;
        double acce_x;
        double acce_y;
        double acce_z;
        void GetMsg(const sensor::imu &msg) 
        {
            header = msg.header;
            tag = msg.tag;
            gyro_x = msg.gyro_x;
            gyro_y = msg.gyro_y;
            gyro_z = msg.gyro_z;
            acce_x = msg.acce_x;
            acce_y = msg.acce_y;
            acce_z = msg.acce_z;
        }
    };

    struct FusionPose
    {
        std_msgs::Header header;
        double latitude;
        double longitude;
        Vector3d position;
        double vel_fusion;
        Vector3d vel_odom;
        Vector3d attitude;
        Vector3d position_dr;
        Vector3d angular;  //deg/s
        Vector3d acceler;  //m/s^2
        Vector3d abias;
        Vector3d gbias;
        Vector3d states;
        Vector4d offset;
        double drtime;
        Vector4i lidarcnt;
        Vector4i gnsscnt;
        Vector3i odomcnt;
        Vector3d gbias_init;
        STATUS mode;
        STATUS status;
        STATUS vehicle;
        double gyrovar;
        double accevar;
    };

    class preprocess
    {
    public:
        preprocess();
        ~preprocess();


    private:

        


    };
}
}




#endif