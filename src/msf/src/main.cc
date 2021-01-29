#include "fusion/fusion.h"
using namespace msf::fusion;

int main(int argc, char **argv)                         
{   
    msfusion msfapp_;                                                   
    signal(SIGINT, app_sigint_handler);                                          
    ros::init(argc, argv, msfapp_.name());              
    msfapp_.spin();                                     
    return 0;                                           
}

