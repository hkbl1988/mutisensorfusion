#include "fusion.h"

namespace msf
{
    namespace fusion
    {
        int msfusion::init()
        {
            setThreadNum(4);
            return 0;
        }

        string msfusion::name()
        {
            return "msfusion";
        }

        int msfusion::start()
        {
            bfusion_.reset(new localfusion());
            if(bfusion_ == nullptr) return -1;
            return bfusion_->start();
        }

        void msfusion::stop()
        {
            bfusion_->stop();
        }
    }
}