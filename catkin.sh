#!/bin/bash
# version="v0.0.1 first version: add test functions. By:gqp."
# version="v0.0.2 Change to AVOS_X version. By:gqp."
source ~/.bashrc

catkin_make -DCMAKE_BUILD_TYPE=Release --pkg sensor
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg msfusion

