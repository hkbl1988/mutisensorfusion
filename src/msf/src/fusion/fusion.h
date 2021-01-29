#ifndef FUSION_H
#define FUSION_H

#include "appfusion.h"
#include "localfusion.h"

namespace msf
{
    namespace fusion
    {
        class msfusion : public appfusion
        {
            public:
                int init() override;

                string name() override;

                int start() override;

                void stop() override;

            private:
                unique_ptr<basefusion> bfusion_;
        };
    }
}

#endif