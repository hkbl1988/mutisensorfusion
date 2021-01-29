#include "threadinteg.h"

namespace msf
{
    namespace fusion
    {
        namespace integ
        {
            ThreadInteg::ThreadInteg()
            :   keep_lidar_running_(true),
                lidar_queue_max_size_(50),
                keep_gnss_running_(true),
                gnss_queue_max_size_(50),
                keep_odom_running_(true),
                odom_queue_max_size_(50),
                keep_imu_running_(true),
                imu_queue_max_size_(200) {}

            ThreadInteg::~ThreadInteg()
            {
                StopThread();
            }

            int ThreadInteg::init()
            {
                msf_integ_.reset(new ProcessInteg());
                if(msf_integ_ == nullptr) return -1;
                StartThread();
                return 0;
            }

            void ThreadInteg::LidarProcess(const LidarPose &pose)
            {
                std::unique_lock<std::mutex> lock(lidar_data_queue_mutex_);
                lidar_data_queue_.push(pose);
                size_t size = lidar_data_queue_.size();
                while (size > lidar_queue_max_size_) 
                {
                    lidar_data_queue_.pop();
                    --size;
                }
                lidar_data_signal_.notify_one();
                return;
            }

            void ThreadInteg::GnssProcess(const GnssPose &pose)
            {
                std::unique_lock<std::mutex> lock(gnss_data_queue_mutex_);
                gnss_data_queue_.push(pose);
                size_t size = gnss_data_queue_.size();
                while (size > gnss_queue_max_size_) 
                {
                    gnss_data_queue_.pop();
                    --size;
                }
                gnss_data_signal_.notify_one();
                return;
            }

            void ThreadInteg::OdomProcess(const OdomPose &pose)
            {
                std::unique_lock<std::mutex> lock(odom_data_queue_mutex_);
                odom_data_queue_.push(pose);
                size_t size = odom_data_queue_.size();
                while (size > odom_queue_max_size_) {
                    odom_data_queue_.pop();
                    --size;
                }
                odom_data_signal_.notify_one();
            }

            
            void ThreadInteg::ImuProcess(const ImuPose &pose)
            {
                std::unique_lock<std::mutex> lock(imu_data_queue_mutex_);
                imu_data_queue_.push(pose);
                size_t size = imu_data_queue_.size();
                while (size > imu_queue_max_size_) {
                    imu_data_queue_.pop();
                    --size;
                }
                imu_data_signal_.notify_one();
            }

            void ThreadInteg::LidarThread()
            {
                while (keep_lidar_running_.load()) 
                {
                    {
                        std::unique_lock<std::mutex> lock(lidar_data_queue_mutex_);
                        if (lidar_data_queue_.size() == 0) 
                        {
                            lidar_data_signal_.wait(lock);
                            continue;
                        }
                    }
                    LidarPose lidar_data;
                    int waiting_num = 0;
                    {
                        std::unique_lock<std::mutex> lock(lidar_data_queue_mutex_);
                        lidar_data = lidar_data_queue_.front();
                        lidar_data_queue_.pop();
                        waiting_num = lidar_data_queue_.size();
                    }
                    if (waiting_num > 2) ROS_WARN_STREAM(waiting_num << " lidar msg are waiting to process.");
                    msf_integ_->addmeasure(lidar_data);
                }

                ROS_INFO_STREAM("Exited lidar data process thread");
            }
            void ThreadInteg::GnssThread()
            {
                while (keep_gnss_running_.load()) 
                {
                    {
                        std::unique_lock<std::mutex> lock(gnss_data_queue_mutex_);
                        if (gnss_data_queue_.size() == 0) 
                        {
                            gnss_data_signal_.wait(lock);
                            continue;
                        }
                    }
                    GnssPose gnss_data;
                    int waiting_num = 0;
                    {
                        std::unique_lock<std::mutex> lock(gnss_data_queue_mutex_);
                        gnss_data = gnss_data_queue_.front();
                        gnss_data_queue_.pop();
                        waiting_num = gnss_data_queue_.size();
                    }
                    if (waiting_num > 2) ROS_WARN_STREAM(waiting_num << " gnss msg are waiting to process.");
                    ROS_INFO_STREAM_ONCE("AddGnssMeasure");
                    msf_integ_->addmeasure(gnss_data);
                }
                ROS_INFO_STREAM("Exited gnss data process thread");
            }

            void ThreadInteg::OdomThread()
            {
                while (keep_odom_running_.load()) 
                {
                    {
                        std::unique_lock<std::mutex> lock(odom_data_queue_mutex_);
                        if (odom_data_queue_.size() == 0) 
                        {
                            odom_data_signal_.wait(lock);
                            continue;
                        }
                    }
                    OdomPose odom_data;
                    int waiting_num = 0;
                    {
                        std::unique_lock<std::mutex> lock(odom_data_queue_mutex_);
                        odom_data = odom_data_queue_.front();
                        odom_data_queue_.pop();
                        waiting_num = odom_data_queue_.size();
                    }

                    if (waiting_num > 2) ROS_WARN_STREAM(waiting_num << " odom msg are waiting to process.");
                    
                    ROS_INFO_STREAM_ONCE("AddOdomMeasure");
                    msf_integ_->addmeasure(odom_data);
                }
                ROS_INFO_STREAM("Exited odom data process thread");
            }

            void ThreadInteg::ImuThread()
            {
                while (keep_imu_running_.load()) 
                {
                    {
                        std::unique_lock<std::mutex> lock(imu_data_queue_mutex_);
                        if (imu_data_queue_.size() == 0) 
                        {
                            imu_data_signal_.wait(lock);
                            continue;
                        }
                    }
                    ImuPose imu_data;
                    int waiting_num = 0;
                    {
                        std::unique_lock<std::mutex> lock(imu_data_queue_mutex_);
                        imu_data = imu_data_queue_.front();
                        imu_data_queue_.pop();
                        waiting_num = imu_data_queue_.size();
                    }
                    if (waiting_num > 2) ROS_WARN_STREAM(waiting_num << " imu msg are waiting to process.");
                    ROS_INFO_STREAM_ONCE("AddImuData");
                    msf_integ_->addimu(imu_data);
                }
                ROS_INFO_STREAM("Exited imu data process thread");
            }

            FusionPose ThreadInteg::GetFusion(void)
            {
                return msf_integ_->getfusion();
            }

            void ThreadInteg::SetMode(STATUS mode)
            {
                msf_integ_->setmode(mode);
            }

            void ThreadInteg::Reset()
            {
                msf_integ_->reset();
            }

            void ThreadInteg::StartThread()
            {
                keep_lidar_running_ = true;
                const auto &lidar_loop_func = [this] { LidarThread(); };
                lidar_data_thread_ = std::thread(lidar_loop_func);

                keep_gnss_running_ = true;
                const auto &gnss_loop_func = [this] { GnssThread(); };
                gnss_data_thread_ = std::thread(gnss_loop_func);

                keep_odom_running_ = true;
                const auto &odom_loop_func = [this] { OdomThread(); };
                odom_data_thread_ = std::thread(odom_loop_func);

                keep_imu_running_ = true;
                const auto &imu_loop_func = [this] { ImuThread(); };
                imu_data_thread_ = std::thread(imu_loop_func);
            }

            void ThreadInteg::StopThread()
            {
                if (keep_lidar_running_.load()) 
                {
                    keep_lidar_running_ = false;
                    lidar_data_signal_.notify_one();
                    lidar_data_thread_.join();
                }

                if (keep_gnss_running_.load()) 
                {
                    keep_gnss_running_ = false;
                    gnss_data_signal_.notify_one();
                    gnss_data_thread_.join();
                }

                if (keep_odom_running_.load()) 
                {
                    keep_odom_running_ = false;
                    odom_data_signal_.notify_one();
                    odom_data_thread_.join();
                }

                if (keep_imu_running_.load()) 
                {
                    keep_imu_running_ = false;
                    imu_data_signal_.notify_one();
                    imu_data_thread_.join();
                }
            }
        }
    }
}