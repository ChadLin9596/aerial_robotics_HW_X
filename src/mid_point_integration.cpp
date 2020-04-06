#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/Path.h>

// declare publisher
ros::Publisher pub_pos;

// declare msgs
nav_msgs::Path result;

// delcare global variable
Eigen::Vector3d G(0, 0, 9.81);
double current_time = -1;
Eigen::Vector3d acc0, gyr0;
Eigen::Vector3d position(0, 0, 0), velocity(0, 0, 0);
Eigen::Quaterniond orientation(1, 0, 0, 0);

void cb_imu(const sensor_msgs::Imu::ConstPtr& msg){
    sensor_msgs::Imu imu = *msg;
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;

    // ========================= //
    // Implement your code below //
    if (current_time < 0){
        current_time = msg->header.stamp.toSec();
        acc0 << msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z;
        gyr0 << msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z;
    }
    double dt = msg->header.stamp.toSec() - current_time;
    current_time = msg->header.stamp.toSec();
    Eigen::Vector3d acc1(msg->linear_acceleration.x,
                         msg->linear_acceleration.y,
                         msg->linear_acceleration.z);
    Eigen::Vector3d gyr1(msg->angular_velocity.x,
                         msg->angular_velocity.y,
                         msg->angular_velocity.z);
    Eigen::Vector3d un_acc_0 = orientation * acc0 - G;
    Eigen::Vector3d un_gyr = 0.5 * (gyr0 + gyr1);
    Eigen::Quaterniond deltaQ(1,
                              0.5 * un_gyr(0) * dt,
                              0.5 * un_gyr(1) * dt,
                              0.5 * un_gyr(2) * dt);
    orientation = orientation * deltaQ;
    Eigen::Vector3d un_acc_1 = orientation * acc1 - G;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    position = position + dt * velocity + 0.5 * dt * dt * un_acc;
    velocity = velocity + dt * un_acc;
    acc0 = acc1;
    gyr0 = gyr1;
    pose.pose.position.x = position(0);
    pose.pose.position.y = position(1);
    pose.pose.position.z = position(2);
    pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y();
    pose.pose.orientation.z = orientation.z();
    pose.pose.orientation.w = orientation.w();
    // ========================= //

    // publish msgs
    result.header = msg->header;
    result.poses.push_back(pose);
    pub_pos.publish(result);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid_point_integration");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("My Student ID is : " << "0XXXXXX");

  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu> ("/mavros/imu/data_raw", 2, cb_imu);
  pub_pos = nh.advertise<nav_msgs::Path> ("/pos", 2);

  ros::spin();
}
