#ifndef APP_FUSION_H_
#define APP_FUSION_H_

#include <csignal>
#include <string>
#include <memory>
#include <vector>

#include "ros/ros.h"

using namespace std;

namespace msf {
    namespace fusion {

        class appfusion
        {
        public:

            virtual int spin();

            virtual ~appfusion() = default;

            void setThreadNum(uint32_t thread_num);

        protected:

            virtual int init() = 0;

            virtual string name() = 0;

            virtual int start() = 0;

            virtual void stop() = 0;

            uint32_t thread_num_ = 1;

        };

        void app_sigint_handler(int signal_num);

    }
}
#endif