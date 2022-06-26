# ROS IMU Bias Correction

```
mkdir imu_ws && cd imu_ws
mkdir src && cd src
git clone git@github.com:vishrutskaushik/ros-imu-bias-correction.git
cd .. && catkin_make
source devel/setup.bash
rosrun imu_calibrate imu_calibrate_node
```
