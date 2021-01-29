#ifndef THRED_INTEG_H
#define THRED_INTEG_H

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include "processinteg.h"
#include "preprocess.h"

using namespace msf::common;

namespace msf
{
    namespace fusion
    {
        namespace integ
        {
            
            class ThreadInteg
            {
            public:
                ThreadInteg();
                ~ThreadInteg();
            
                int init();

                void LidarProcess(const LidarPose &pose);
                void OdomProcess(const OdomPose &pose);
                void GnssProcess(const GnssPose &pose);
                void ImuProcess(const ImuPose &pose);
                FusionPose GetFusion();
                void SetMode(STATUS mode);
                void Reset();

            private:
                void StartThread();
                void StopThread();

                //sensor data thread
                void LidarThread();
                void GnssThread();
                void ImuThread();
                void OdomThread();

            private:
                std::unique_ptr<BaseInteg> msf_integ_;

                // lidar  process real time thread
                std::atomic<bool> keep_lidar_running_;
                std::thread lidar_data_thread_;
                std::condition_variable lidar_data_signal_;
                std::queue<LidarPose> lidar_data_queue_;
                size_t lidar_queue_max_size_;
                std::mutex lidar_data_queue_mutex_;

                // gnss process real time thread
                std::atomic<bool> keep_gnss_running_;
                std::thread gnss_data_thread_;
                std::condition_variable gnss_data_signal_;
                std::queue<GnssPose> gnss_data_queue_;
                size_t gnss_queue_max_size_;
                std::mutex gnss_data_queue_mutex_;

                // odom process thread
                std::atomic<bool> keep_odom_running_;
                std::thread odom_data_thread_;
                std::condition_variable odom_data_signal_;
                std::queue<OdomPose> odom_data_queue_;
                size_t odom_queue_max_size_;
                std::mutex odom_data_queue_mutex_;

                // imu process thread
                std::atomic<bool> keep_imu_running_;
                std::thread imu_data_thread_;
                std::condition_variable imu_data_signal_;
                std::queue<ImuPose> imu_data_queue_;
                size_t imu_queue_max_size_;
                std::mutex imu_data_queue_mutex_;

                std::mutex integ_operation_mutex_;

            };
        }
    }
}



#endif