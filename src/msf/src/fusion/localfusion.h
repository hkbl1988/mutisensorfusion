
#ifndef LOCAL_FUSION_H
#define LOCAL_FUSION_H


#include "basefusion.h"
#include "preprocess.h"
#include "params.h"
#include "threadinteg.h"
#include "sensor/result.h"

using namespace msf::fusion::integ;

namespace msf
{
    namespace fusion
    {
        class localfusion : public msf::fusion::basefusion
        {
        public:
            localfusion();
            ~localfusion();

            int start() override;
            int stop() override;

        private:
            int init();
            void OnLidar(const sensor::lidar &msg);
            void OnGnss(const sensor::gnss &msg);
            void OnImu(const sensor::imu &msg);
            void OnOdom(const sensor::odom &msg);

            void OnResultTimer();

            sensor::result FusionToMsg(const FusionPose &pose);


        private:
            ros::NodeHandle nh_;
            std::vector<ros::Subscriber> subscribe_;

            ros::Publisher msf_pub_;

            GnssPose gnsspose_;
            LidarPose lidarpose_;
            ImuPose imupose_;
            OdomPose odompose_;

            msf::common::params conf_;
            std::thread pub_data_timer_;

            ThreadInteg threadinteg_;
            
        };
    }
}



#endif