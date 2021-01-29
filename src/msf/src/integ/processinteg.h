#ifndef PROCESS_INTEG_H
#define PROCESS_INTEG_H

#include "baseinteg.h"
#include "mutifusion.h"

using namespace msf::common;

namespace msf
{
    namespace fusion
    {
        namespace integ
        {
            class ProcessInteg : public BaseInteg
            {
            public:
                ProcessInteg(/* args */);
                ~ProcessInteg();
                int init();
                void addmeasure(LidarPose &lidar) override;
                void addmeasure(GnssPose &gnss) override;
                void addmeasure(OdomPose &odom) override;
                void addimu(ImuPose &imu) override;
                FusionPose getfusion(void) override;
                void setmode(STATUS mode) override;
                void reset() override;

            private:

                mutifusion msf_;
            };
        }
    }
} // namespace name



#endif