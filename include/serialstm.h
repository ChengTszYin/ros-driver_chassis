#ifndef SERIALSTM_H
#define SERIALSTM_H
#include<iostream>
#include<cmath>
#include <serial/serial.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <stm_driver/Wheel.h>
#include <sensor_msgs/Range.h> 
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int8MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <deque>

using namespace std;

struct Hostmessage 
{
    uint8_t HleftID = 0x01;
    uint8_t HrightID = 0x02;
    uint8_t LleftID = 0x03;
    uint8_t LrightID = 0x04;
    int Hleftspeed = 0;
    int Hrightspeed = 0;
    int Lleftspeed = 0;
    int Lrightspeed = 0;
};

struct recvMessage
{
    uint8_t HleftID = 0x01;
    uint8_t HrightID = 0x02;
    int16_t Hleftspeed = 0;
    int16_t Hrightspeed = 0;
    uint8_t LleftID = 0x01;
    uint8_t LrightID = 0x02;
    int16_t Lleftspeed = 0;
    int16_t Lrightspeed = 0;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z; 
    int16_t Q0;
    int16_t Q1;
    int16_t Q2;
    int16_t Q3;
    int16_t sensor1;
    int16_t sensor2;
    uint8_t d80nk1;
    uint8_t d80nk2;
    uint8_t d80nk3;
    uint8_t d80nk4;
};

class SerialSTM {
    private:
        string port = "COM4";
        int baud = 115200;
        serial::Serial ser;
        ros::NodeHandle n_ser;
        double vel_dt_ = 0.0;
        double d_lv = 0.0;
        double d_rv = 0.0;
        double dxy_ave = 0;
        double dth = 0.0;
        double vxy = 0.0;
        double vth = 0.0;
        double dx = 0.0;
        double dy = 0.0;
        double x_pos = 0.0;
        double y_pos = 0.0;
        double th = 0.0;
        ros::Time last_vel_time_;
        // geometry_msgs::Vector3Stamped speed_msgs;
        stm_driver::Wheel speed_msgs;
        sensor_msgs::Range front_dist;
        sensor_msgs::Range back_dist;
        sensor_msgs::Imu imu_msgs;
        std_msgs::Int8MultiArray wsad;
        ros::Publisher  ser_pub;
        ros::Publisher  front_pub;
        ros::Publisher  back_pub;
        ros::Publisher  imu_pub;
        ros::Publisher  wsad_pub;
        ros::Publisher  odom_pub;
        tf::TransformBroadcaster broadcaster;

    public:
        SerialSTM(string port, int baud);
        void readSpeed(recvMessage* recvmsg, uint8_t* bufferArray);
        void putSpeed(Hostmessage* hostmsg);
        void speedPublish(recvMessage* recvmsg, double time);
        void IMUPublish(recvMessage* recvmsg);
        void distancePublish(recvMessage* recvmsg);
        void bumpPublish(recvMessage* recvmsg);
        int notopen(std::string &result);
        uint8_t getcrc(uint8_t* Bytecode, int len);
};



#endif

