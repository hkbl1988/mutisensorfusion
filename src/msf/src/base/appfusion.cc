#include "appfusion.h"

namespace msf {
    namespace fusion {

        void appfusion::setThreadNum(uint32_t thread_num)
        {
            if(thread_num>1) thread_num_ = thread_num;
        }

        int appfusion::spin()
        {
            auto status = init();
            if(status == -1) return -1;

            unique_ptr<ros::AsyncSpinner> spinner;
            if(thread_num_ > 1)
            {
                spinner = unique_ptr<ros::AsyncSpinner>(
                    new ros::AsyncSpinner(thread_num_));
            }
            
            status = start();
            if(status == -1) return -2;

            if(spinner) spinner->start();
            else        ros::spin();

            ros::waitForShutdown();
            stop();
            return 0;
        }

        void app_sigint_handler(int signal_num)
        {
            if(signal_num != SIGINT) return;
            bool static stopping = false;
            if(stopping) return;
            stopping = true;
            ros::shutdown();
        }
    }
}