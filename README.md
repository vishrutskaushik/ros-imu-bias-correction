# ROS IMU Bias Correction

### This package is used to auto-calibrate the IMU in ROS. 

```
mkdir imu_ws && cd imu_ws
mkdir src && cd src
git clone git@github.com:vishrutskaushik/ros-imu-bias-correction.git
cd .. && catkin_make
source devel/setup.bash
rosrun ros-imu-bias-correction imu_calibrate_node
```

#### This program assumes that the biased IMU data is published on the topic "/imu/raw_data" and publishes the ubiased data on "/imu/data". 

#### If your robot or rosbag does not publish data on /imu/raw_data you can easily change it in src/imu_calibrate_node.cpp on line 105.
