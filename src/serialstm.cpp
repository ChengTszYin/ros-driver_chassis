#include "serialstm.h"
const int WINDOW_SZ = 5;
const double LOOPTIME = 100;

SerialSTM::SerialSTM(string port, int baud) : port(port), baud(baud)
{   
    ser.setPort(port);
    ser.setBaudrate(baud);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    try
    {
        ser.open();
    }

    catch(const std::exception& e )
    {
        cout << "Unable to open port: " << e.what() << endl;
        throw;
    }
    // ser_pub = n_ser.advertise<geometry_msgs::Vector3Stamped> ("speed", 1000);
    ser_pub = n_ser.advertise<stm_driver::Wheel>("speed", 1000);
    front_pub = n_ser.advertise<sensor_msgs::Range> ("front_dist", 1000);
    back_pub = n_ser.advertise<sensor_msgs::Range> ("back_dist", 1000);
    imu_pub = n_ser.advertise<sensor_msgs::Imu> ("imu", 1000);
    wsad_pub = n_ser.advertise<std_msgs::Int8MultiArray> ("WSAD", sizeof(wsad));
    odom_pub = n_ser.advertise<nav_msgs::Odometry>("odom", 1000); 
}

uint8_t SerialSTM::getcrc(uint8_t* Bytecode, int len)
{
    uint8_t sum = 0;
    for(int i=0; i<len; i++)
    {
        sum += Bytecode[i];
    }
    return sum;
}


int SerialSTM::notopen(std::string &result)
{
    if(!ser.isOpen())
    {
        cout << "serial port is not opened" << endl;
        return 0;
    }
    result = ser.read( ser.available() );
    return 0;
}

void SerialSTM::readSpeed(recvMessage* recvmsg, uint8_t* bufferArray)
{   
    if(!ser.isOpen())
    {
        cout << "serial port is not opened" << endl; 
        return;
    }
    recvmsg->HleftID = bufferArray[1];
    recvmsg->Hleftspeed = (int16_t)((bufferArray[2] << 8) | (bufferArray[3] & 0xFF));
    recvmsg->HrightID = bufferArray[4] & 0xFF;
    recvmsg->Hrightspeed = (int16_t)((bufferArray[5] << 8) | (bufferArray[6] & 0xFF));
    recvmsg->LleftID = bufferArray[7];
    recvmsg->Lleftspeed = (int16_t)((bufferArray[8] << 8) | (bufferArray[9] & 0xFF));
    recvmsg->LrightID = bufferArray[10] & 0xFF;
    recvmsg->Lrightspeed = (int16_t)((bufferArray[11] << 8) | (bufferArray[12] & 0xFF));
    recvmsg->acc_x = ((bufferArray[13] << 8) &  0xFF) | (bufferArray[14] & 0xFF);
    recvmsg->acc_y = ((bufferArray[15] << 8) &  0xFF) | (bufferArray[16] & 0xFF);
    recvmsg->acc_z = ((bufferArray[17] << 8) &  0xFF) | (bufferArray[18] & 0xFF);
    recvmsg->gyro_x = ((bufferArray[19] << 8) &  0xFF) | (bufferArray[20] & 0xFF);
    recvmsg->gyro_y = ((bufferArray[21] << 8) &  0xFF) | (bufferArray[22] & 0xFF);
    recvmsg->gyro_z = ((bufferArray[23] << 8) &  0xFF) | (bufferArray[24] & 0xFF);
    recvmsg->Q0 = ((bufferArray[25] << 8) &  0xFF) | (bufferArray[26] & 0xFF);
    recvmsg->Q1 = ((bufferArray[27] << 8) &  0xFF) | (bufferArray[28] & 0xFF);
    recvmsg->Q2 = ((bufferArray[29] << 8) &  0xFF) | (bufferArray[30] & 0xFF);
    recvmsg->Q3 = ((bufferArray[31] << 8) &  0xFF) | (bufferArray[32] & 0xFF);
    recvmsg->sensor1 = ((bufferArray[33] << 8) & 0xFF) | (bufferArray[34] & 0xFF);
    recvmsg->sensor2 = ((bufferArray[35] << 8) & 0xFF) | (bufferArray[36] & 0xFF);
    recvmsg->d80nk1 = (bufferArray[37]);
    recvmsg->d80nk2 = (bufferArray[38]);
    recvmsg->d80nk3 = (bufferArray[39]);
    recvmsg->d80nk4 = (bufferArray[40]);
}

void SerialSTM::speedPublish(recvMessage* recvmsg, double time)
{
    
    ros::Time current_time = ros::Time::now();
    int hl_speed = recvmsg -> Hleftspeed;
    int hr_speed = recvmsg -> Hrightspeed;
    int ll_speed = recvmsg -> Lleftspeed;
    int lr_speed = recvmsg -> Lrightspeed;

    cout << "hl_speed: " << " " << hl_speed << endl;
    cout << "hr_speed: " << " " << hr_speed << endl;
    cout << "ll_speed: " << " " << ll_speed << endl;
    cout << "lr_speed: " << " " << lr_speed << endl;

    speed_msgs.header.stamp = ros::Time::now();
    speed_msgs.TopLeftWheel = lr_speed;
    speed_msgs.TopRightWheel = ll_speed;
    speed_msgs.BottomLeftWheel = hr_speed;
    speed_msgs.BottomRightWheel = hl_speed;
    speed_msgs.time = time / 1000;
    ser_pub.publish(speed_msgs);
}

void SerialSTM::IMUPublish(recvMessage* recvmsg)
{
    imu_msgs.linear_acceleration.x = recvmsg -> acc_x;
    imu_msgs.linear_acceleration.y = recvmsg -> acc_y;
    imu_msgs.linear_acceleration.z = recvmsg -> acc_z;
    imu_msgs.angular_velocity.x = recvmsg -> gyro_x;
    imu_msgs.angular_velocity.y = recvmsg -> gyro_y;
    imu_msgs.angular_velocity.z = recvmsg -> gyro_z;
    imu_msgs.header.stamp = ros::Time::now();
    imu_pub.publish(imu_msgs);
}

void SerialSTM::distancePublish(recvMessage* recvmsg)
{
    front_dist.min_range = 20.0;
    front_dist.max_range = 720.0;
    front_dist.range = recvmsg -> sensor1;
    front_dist.header.stamp = ros::Time::now();
    front_pub.publish(front_dist);

    back_dist.min_range = 20.0;
    back_dist.max_range = 720.0;
    back_dist.range = recvmsg -> sensor2;
    back_dist.header.stamp = ros::Time::now();
    back_pub.publish(back_dist);
}

void SerialSTM::bumpPublish(recvMessage* recvmsg)
{
    wsad.data = {recvmsg->d80nk1 - 48, recvmsg->d80nk2 - 48, recvmsg->d80nk3 - 48, recvmsg->d80nk4 - 48};
    wsad_pub.publish(wsad);
}


void SerialSTM::putSpeed(Hostmessage* hostmsg)
{
    if(!ser.isOpen())
    {
        cout << "serial port is not opened" << endl;
        return;
    }
    uint8_t sendByte[14];
    sendByte[0] = 0x00;
    sendByte[1] = hostmsg->HleftID;
    sendByte[2] = (hostmsg->Hleftspeed >> 8) & 0xFF;
    sendByte[3] = hostmsg->Hleftspeed & 0xFF;
    sendByte[4] = hostmsg->HrightID;
    sendByte[5] = (hostmsg->Hrightspeed >> 8) & 0xFF;
    sendByte[6] = hostmsg->Hrightspeed & 0xFF;
    sendByte[7] = hostmsg->LleftID;
    sendByte[8] = (hostmsg->Lleftspeed >> 8) & 0xFF;
    sendByte[9] = hostmsg->Lleftspeed & 0xFF;
    sendByte[10] = hostmsg->LrightID;
    sendByte[11] = (hostmsg->Lrightspeed >> 8) & 0xFF;
    sendByte[12] = hostmsg->Lrightspeed & 0xFF;
    sendByte[13] = getcrc(sendByte, 13);
    ser.write(sendByte, sizeof(sendByte));
}

