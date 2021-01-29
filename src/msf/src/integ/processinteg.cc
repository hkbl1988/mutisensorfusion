#include "processinteg.h"

namespace msf
{
    namespace fusion
    {
        namespace integ
        {
            ProcessInteg::ProcessInteg(){}

            ProcessInteg::~ProcessInteg(){}

            void ProcessInteg::addmeasure(LidarPose &lidar)
            {
                msf_.SetLidarPose(lidar);
            }
            void ProcessInteg::addmeasure(GnssPose &gnss)
            {
                msf_.SetGnssPose(gnss);
            }
            void ProcessInteg::addmeasure(OdomPose &odom) 
            {
                msf_.SetOdomPose(odom);
            }
            void ProcessInteg::addimu(ImuPose &imu) 
            {
                msf_.SetImuPose(imu);
                msf_.ApplyNav();
            } 
            FusionPose ProcessInteg::getfusion(void) 
            {
                FusionPose fpose;
                auto result = msf_.GetFusionResult();
                //trans result -> fusionpose

                fpose.header = result.header;
                fpose.position = result.position;
                fpose.mode = result.workmode;
                fpose.abias = result.abias;
                fpose.gbias = result.gbias;
                fpose.gbias_init = result.gbias_init;
                fpose.states = result.states;
                fpose.lidarcnt = result.lidarcnt;
                for(int i=0;i<4;i++)fpose.gnsscnt(i) = result.gnsscnt(i);
                fpose.odomcnt = result.odomcnt;
                fpose.offset(0) = result.yawoffset;
                fpose.offset(1) = result.posoffset(0);
                fpose.offset(2) = result.posoffset(1);
                fpose.offset(3) = result.posoffset(2);
                fpose.attitude = result.attitude * kRAD2DEG;
                fpose.angular = result.angular * kRAD2DEG;
                fpose.vehicle = result.vehicle_status;
                fpose.vel_fusion = result.speed;
                fpose.vel_odom = result.velodom;
                fpose.gyrovar = result.gyro_var_norm;
                fpose.accevar = result.acce_var_norm;
                fpose.acceler = result.acceler;

                
                return fpose;
            }
            void ProcessInteg::setmode(STATUS mode) 
            {
                msf_.ResetFusionMode(mode);
            }
            void ProcessInteg::reset() 
            {
                msf_.ResetFilter();
            }
        }
    }
}