#ifndef BASE_INTEG_H
#define BASE_INTEG_H

#include "cmn.h"
#include "preprocess.h"

using namespace msf::common;

namespace msf
{
    namespace fusion
    {
        namespace integ
        {
            class BaseInteg
            {
            public:
                virtual ~BaseInteg() = default;
                virtual void addmeasure(LidarPose &lidar) = 0;
                virtual void addmeasure(GnssPose &gnss) = 0;
                virtual void addmeasure(OdomPose &odom) = 0;
                virtual void addimu(ImuPose &imu) = 0;
                virtual FusionPose getfusion(void) = 0;
                virtual void setmode(STATUS mode) = 0;
                virtual void reset() = 0;
            };            
        }
    }
}


#endif