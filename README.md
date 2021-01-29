节点运行流程

1.运行./catkin.sh 编译节点；

2.启动roscore

3.cd bag 并且 rosbag play test.bag

4.运行节点 source devel/setup.bash 然后 rosrun msfusion msfusion

5.rostopic echo /msfpose 获取融合定位结果


本程序适用于低速运行车辆，采用22维ESKF模型融合IMU、轮速、GNSS以及激光定位等传感器，输出定位结果

可使用MATLAB读取bag进行数据分析

如有疑问/MATLAB脚本可联系:hkbl1988bb@163.com
