#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <chrono>


class IMU_Calibrate{
    public:
    double total_acc_x_ = 0.0;
    double total_acc_y_ = 0.0;
    double total_acc_z_ = 0.0;
    double total_orientation_x_ = 0.707;
    double total_orientation_y_ = 0.0;
    double total_orientation_z_ = 0.707;
    double total_orientation_w_ = 0.0;

    double bias_acc_x_ = 0.0;
    double bias_acc_y_ = 0.0;
    double bias_acc_z_ = 0.0;
    double bias_orientation_x_ = 0.0;
    double bias_orientation_y_ = 0.0;
    double bias_orientation_z_ = 0.0;
    double bias_orientation_w_ = 0.0;


    double desired_acc_x_ = 0.0;
    double desired_acc_y_ = 0.0;
    double desired_acc_z_ = 9.8;
    double desired_orientation_x_ = 0.0;
    double desired_orientation_y_ = 0.0;
    double desired_orientation_z_ = 0.0;
    double desired_orientation_w_ = 1.0;
    
    long long int count = 0; 

    ros::Publisher imuPublisher_;
    sensor_msgs::Imu unBiasedIMU_;

        IMU_Calibrate(ros::NodeHandle &nh){
            imuPublisher_ = nh.advertise<sensor_msgs::Imu>("/imu/data", 1000);
        }

        void readIMUData(const sensor_msgs::Imu::ConstPtr &);
        void removeIMUBias(const sensor_msgs::Imu::ConstPtr &);
        void calculateIMUBias(const sensor_msgs::Imu::ConstPtr &);
        long long int getCount(){return count;}

};

void IMU_Calibrate::readIMUData(const sensor_msgs::Imu::ConstPtr & msg){
    ROS_INFO("IMU Sequence: [%d]", msg->header.seq);
    ROS_INFO("IMU Orientation: x = [%f], y = [%f], z = [%f], w = [%f]", msg->orientation.x, msg->orientation.y, 
                                                                        msg->orientation.y, msg->orientation.w);
    ROS_INFO("IMU Acceleration: x = [%f], y = [%f], z = [%f]", msg->linear_acceleration.x, msg->linear_acceleration.y,
                                                               msg->linear_acceleration.z);
}

void IMU_Calibrate::calculateIMUBias(const sensor_msgs::Imu::ConstPtr & msg){
    count += 1;

    total_acc_x_ += msg->linear_acceleration.x;
    total_acc_y_ += msg->linear_acceleration.y;
    total_acc_z_ += msg->linear_acceleration.z;
    total_orientation_x_ += msg->orientation.x;
    total_orientation_y_ += msg->orientation.y;
    total_orientation_z_ += msg->orientation.z;
    total_orientation_w_ += msg->orientation.w;

    bias_acc_x_ = desired_acc_x_ - (total_acc_x_ / count);
    bias_acc_y_ = desired_acc_y_ - (total_acc_y_ / count);
    bias_acc_z_ = desired_acc_z_ - (total_acc_z_ / count);
    bias_orientation_x_ = desired_orientation_x_ - (total_orientation_x_ / count);
    bias_orientation_y_ = desired_orientation_y_ - (total_orientation_y_ / count);
    bias_orientation_z_ = desired_orientation_z_ - (total_orientation_z_ / count);
    bias_orientation_w_ = desired_orientation_w_ - (total_orientation_w_ / count);

    // ROS_INFO("IMU Bias: \nAcceleration: x = [%f], y = [%f], z = [%f]\nOrientation: x = [%f], y = [%f], z = [%f], w = [%f]", 
    //                                         bias_acc_x_, bias_acc_y_, bias_acc_z_, 
    //                                         bias_orientation_x_, bias_orientation_y_, bias_orientation_z_, bias_orientation_w_);
}

void IMU_Calibrate::removeIMUBias(const sensor_msgs::Imu::ConstPtr & msg){
    unBiasedIMU_.header = msg->header;
    unBiasedIMU_.angular_velocity = msg->angular_velocity;

    unBiasedIMU_.linear_acceleration.x = msg->linear_acceleration.x + bias_acc_x_;
    unBiasedIMU_.linear_acceleration.y = msg->linear_acceleration.y + bias_acc_y_;
    unBiasedIMU_.linear_acceleration.z = msg->linear_acceleration.z + bias_acc_z_;

    unBiasedIMU_.orientation.x = msg->orientation.x + bias_orientation_x_;
    unBiasedIMU_.orientation.y = msg->orientation.y + bias_orientation_y_;
    unBiasedIMU_.orientation.z = msg->orientation.z + bias_orientation_z_;
    unBiasedIMU_.orientation.w = msg->orientation.w + bias_orientation_w_;

    imuPublisher_.publish(unBiasedIMU_);
}



int main(int argc, char **argv){

    ros::init(argc, argv, "imu_calibrate");
    ros::NodeHandle nh;

    IMU_Calibrate imuCalibrate(nh);


    ros::Subscriber sub = nh.subscribe("/imu/raw_data", 1000, &IMU_Calibrate::calculateIMUBias, &imuCalibrate);

    ROS_INFO("Calibrating IMU, keep the rover stationary");

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    int time_elapsed = 0;

    ros::Rate rate(1);
    while(ros::ok() and (time_elapsed < 10)){
        ros::spinOnce();

        if(imuCalibrate.count == 0){
            begin = std::chrono::steady_clock::now();
            ROS_INFO("Waiting for IMU Data");
            rate.sleep();
            continue;
        }
        end = std::chrono::steady_clock::now();
        time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - begin).count();
    }

    ROS_INFO("IMU Bias: \nAcceleration: x = [%f], y = [%f], z = [%f]\nOrientation: x = [%f], y = [%f], z = [%f], w = [%f]", 
                                            imuCalibrate.bias_acc_x_, imuCalibrate.bias_acc_y_, imuCalibrate.bias_acc_z_, 
                                            imuCalibrate.bias_orientation_x_, imuCalibrate.bias_orientation_y_, imuCalibrate.bias_orientation_z_, imuCalibrate.bias_orientation_w_);


    ROS_INFO("Publishing Unbiased IMU Data");

    sub = nh.subscribe("/imu/raw_data", 1000, &IMU_Calibrate::removeIMUBias, &imuCalibrate);
    ros::spin();

}