#ifndef BASE_FUSION_H
#define BASE_FUSION_H

#include <memory>

namespace msf
{
    namespace fusion
    {
        class basefusion
        {
        public:
            basefusion(){};

            virtual ~basefusion() = default;

            virtual int start() = 0;

            virtual int stop() = 0;

        };
    }
    
} // namespace name



#endif