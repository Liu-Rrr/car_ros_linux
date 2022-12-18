#ifndef CANDRIVE_H
#define CANDRIVE_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include "controlcan.h"
#include "unistd.h"
#define pi 3.1415926

#define uint8 unsigned char
#define uint32 unsigned int
//将uint8型变量合并成uint32型无符号整形变量
#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32)((uint32)((Byte0) & 0x00FF) \
          + ((uint32)((Byte1) & 0x00FF) << 8) \
          + ((uint32)((Byte2) & 0x00FF) << 16) \
          + ((uint32)((Byte3) & 0x00FF) << 24)))

#define left_wheel 1
#define right_wheel 2
struct period_info {
        struct timespec next_period;
        long period_ns;
};
class info_robot
{
public:
    info_robot();
    ~info_robot();
public:
    void periodic_task_init(struct period_info *pinfo);
    void inc_period(struct period_info *pinfo); 
    void do_rt_task();
    void wait_rest_of_period(struct period_info *pinfo);
    void  *simple_cyclic_task(void *data);
    int writespeed(double RobotV,double YawRate);
    float get_motor_pos(int motor_flag);
public:
    double wheel_length;

private:
    VCI_BOARD_INFO pInfo;//用来获取设备信息。
    VCI_BOARD_INFO pInfo1 [50];
    VCI_CAN_OBJ send[2],rec[100];
    int num;
    uint32 pulse_per_circle;
    double wheel_radius;//the radius of the drive wheel is 70mm
private:    
    int speed(int add, int speed_input);
    uint32 place(int add);
};


#endif