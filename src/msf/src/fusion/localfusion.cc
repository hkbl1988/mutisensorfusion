#include "localfusion.h"

namespace msf
{
    namespace fusion
    {
        localfusion::localfusion(){}
        localfusion::~localfusion(){}

        int localfusion::start()
        {
            threadinteg_.init();
            subscribe_.clear();
            subscribe_.push_back(nh_.subscribe("/lidarpose", 10, &localfusion::OnLidar,this));
            subscribe_.push_back(nh_.subscribe("/gnsspose",  10, &localfusion::OnGnss, this));
            subscribe_.push_back(nh_.subscribe("/imuraw",   10, &localfusion::OnImu,  this));
            subscribe_.push_back(nh_.subscribe("/odompose",  10, &localfusion::OnOdom, this));
            
            msf_pub_ = nh_.advertise<sensor::result>("msfpose", 2);

            const auto &pub_timer_func = [this]{ OnResultTimer(); };
            pub_data_timer_ = std::thread(pub_timer_func);
            pub_data_timer_.detach();

            return 0;
        }

        int localfusion::stop()
        {
            return 0;
        }

        void localfusion::OnLidar(const sensor::lidar &msg)
        {
            lidarpose_.GetMsg(msg);
            lidarpose_.yaw = msf::common::Angle2Yaw(lidarpose_.yaw);

            threadinteg_.LidarProcess(lidarpose_);
        }


        void localfusion::OnGnss(const sensor::gnss &msg)
        {
            gnsspose_.GetMsg(msg);
            gnsspose_.xg = gnsspose_.xg - conf_.gnss_.base_x;
            gnsspose_.yg = gnsspose_.yg - conf_.gnss_.base_y;
            gnsspose_.zg = gnsspose_.zg - conf_.gnss_.base_z;
            gnsspose_.yaw = msf::common::LimitYaw(gnsspose_.yaw + conf_.gnss_.angle);

            threadinteg_.GnssProcess(gnsspose_);
        }


        void localfusion::OnImu(const sensor::imu &msg)
        {
            imupose_.GetMsg(msg);
            threadinteg_.ImuProcess(imupose_);
        }


        void localfusion::OnOdom(const sensor::odom &msg)
        {
            odompose_.GetMsg(msg);
            threadinteg_.OdomProcess(odompose_);
        }

        void localfusion::OnResultTimer()
        {
            ros::Rate loop(50);

            while (ros::ok())
            {
                FusionPose pose = threadinteg_.GetFusion();

                sensor::result msg = FusionToMsg(pose);

                static unsigned int seq = 1;
                msg.header.seq = seq;

                msf_pub_.publish(msg);
                seq++;

                
                loop.sleep();
            }
        }

        sensor::result localfusion::FusionToMsg(const FusionPose &pose)
        {
            msf::common::Quaternion quaternion;
            Eigen::Matrix3d cbn;
            sensor::result msg;

            msg.header.stamp = pose.header.stamp;
            msg.header.frame_id = "msfpose";
            msg.latitude = pose.latitude;
            msg.longitude = pose.longitude;

            msg.xg = pose.position(0);
            msg.yg = pose.position(1);
            msg.zg = pose.position(2);
            msg.roll = pose.attitude(0);
            msg.pitch = pose.attitude(1);
            msg.yaw = pose.attitude(2);
            Eigen::Vector3d att(msg.roll,msg.pitch,msg.yaw);
            quaternion.Att2Cbn(cbn,att*kDEG2RAD);
            msg.odomspeed = (cbn.transpose() * pose.vel_odom)(1);
            msg.msfspeed = pose.vel_fusion;

            if      (pose.mode==STATUS::GNSS)           msg.mode = "gnss fusion";
            else if (pose.mode==STATUS::LIDAR)          msg.mode = "lidar fusion";
            else                                        msg.mode = "unkonw fusion";

            if      (pose.status==STATUS::NORMAL)       msg.status = "normal";
            else if (pose.status==STATUS::DEADROCKING)  msg.status = "deadrocking";
            else                                        msg.status = "loss";
    
            if      (pose.vehicle==STATUS::STATIC)      msg.vehicle = "static";
            else if (pose.vehicle==STATUS::FORWARD)     msg.vehicle = "forward";
            else if (pose.vehicle==STATUS::BACKWARD)    msg.vehicle = "backward";
            else                                        msg.vehicle = "unknow"; 
            

            for(int i = 0;i < 3;i++)
            {
                msg.initgbias[i] = pose.gbias_init(i);  //deg/h
                msg.lidarcnt[i+1] = pose.lidarcnt(i+1);
                msg.gnsscnt[i+1] = pose.gnsscnt(i+1);
                msg.odomcnt[i] = pose.odomcnt(i);
                msg.estgbias[i] = pose.gbias(i) * 100;           // deg/h
                msg.estabias[i] = pose.abias(i) * 100;           // mg
                msg.offset[i+1] = pose.offset(i+1) * 100;     // 
            }

            msg.yawerror = pose.states(0);
            msg.kfactor = pose.states(1);
            msg.ptherror = pose.states(2);
            msg.offset[0] = pose.offset(0) * 100;
            msg.gnsscnt[0] = pose.gnsscnt(0);
            msg.lidarcnt[0] = pose.lidarcnt(0);

            msg.drtime = pose.drtime;

            if(pose.drtime >= conf_.fusion_.maxdrtime)
            {
                msg.status = "loss";
            }
            else
            {
                msg.status = "normal";
            }

            double utmx = msg.xg + conf_.gnss_.base_x;
            double utmy = msg.yg + conf_.gnss_.base_y;

            Convert_UTM_To_Geodetic(gnsspose_.zone,gnsspose_.hemisphere,utmx,utmy,&msg.latitude,&msg.longitude);
            
            msg.latitude *= kRAD2DEG;
            msg.longitude *= kRAD2DEG;

            return msg;
        }
    }
}