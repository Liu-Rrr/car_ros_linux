#include "./motor_drive/can_drive.h"



info_robot::info_robot()
{
    //CAN总线初始化
    wheel_radius = 0.08 ;//the radius of the drive wheel is 70mm
    pulse_per_circle = 200000 ;  //轮子转一圈返回的脉冲数
    wheel_length = 0.413 ;  //两个轮子之间的距离

    num=VCI_FindUsbDevice2(pInfo1);
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}
    VCI_INIT_CONFIG config;
	config.AccCode=0x80000008;
	config.AccMask=0xFFFFFFFF;
	config.Filter=0;//接收所有帧
	config.Timing0=0x01;/*波特率250 Kbps  0x03  0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		// printf(">>Init CAN1 error\n");
		std::cout<<"Init CAN1 error"<<std::endl;
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		// printf(">>Start CAN1 error\n");
		std::cout<<"Start CAN1 error"<<std::endl;
		VCI_CloseDevice(VCI_USBCAN2,0);
	}
	
}

info_robot::~info_robot()
{
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
    usleep(100000);//延时100ms。      
    VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
    printf("zdp,awesome!");
}

int info_robot::speed(int add,int  speed_input)
{
	int ret,speed_1,speed_2,speed_3,speed_4;
	speed_input = speed_input * 1000;	//速度单位0.001rpm,乘以1000还原为rpm
	speed_1 = speed_input >> 24;
    speed_2 = speed_input >> 16 & 0x00FF;
    speed_3 = speed_input >> 8 & 0x00FF;
    speed_4 = speed_input & 0x00FF;
	send[0].ID = add;
	send[0].SendType=1;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;
	send[0].Data[0] = 3;
	send[0].Data[1] = 8;
	send[0].Data[2] = 0;
	send[0].Data[3] = speed_4;
	send[0].Data[4] = speed_3;
	send[0].Data[5] = speed_2;
	send[0].Data[6] = speed_1;
	send[0].Data[7] = 0;
	ret = VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
	if (ret != 1)
	{
		std::cout<<"CAN1通道发送失败"<<std::endl;
	}
        
}

uint32 info_robot::place(int add)
{
	int ret;
	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		std::cout<<"Start CAN1 error"<<std::endl;
		VCI_CloseDevice(VCI_USBCAN2,0);
	}
	uint32 data_pos;
	send[1].ID = add;
	send[1].SendType=1;
	send[1].RemoteFlag=0;
	send[1].ExternFlag=0;
	send[1].DataLen=8;
	send[1].Data[0] = 1;
	send[1].Data[1] = 1;
	send[1].Data[2] = 0;
	send[1].Data[3] = 0;
	send[1].Data[4] = 0;
	send[1].Data[5] = 0;
	send[1].Data[6] = 0;
	send[1].Data[7] = 0;
	ret = VCI_Transmit(VCI_USBCAN2, 0, 0, &send[1], 1);
	if (ret != 1)
	{
		printf("CAN1通道发送失败\n");
	}
	ret = VCI_Receive(VCI_USBCAN2,0,0,rec,100,0);
	while (ret<=0)
	{
		ret = VCI_Receive(VCI_USBCAN2,0,0,rec,100,0);
	}
	if (ret > 0)
	{
		data_pos = BUILD_UINT32(rec[0].Data[3],rec[0].Data[4],rec[0].Data[5],rec[0].Data[6]);
	}
	// if (add == 1)
	// {
	// 	printf("motor_pulse is:%d\n",data_pos);
	// }
	
	return data_pos;
}

int info_robot::writespeed(double RobotV_,double YawRate_)
{
	int ret;
	RobotV_ = -RobotV_;
	YawRate_ = YawRate_;
    double right_v = RobotV_ + wheel_length*YawRate_/2;
    double left_v = RobotV_ - wheel_length*YawRate_/2;
    double right_w = int((right_v/wheel_radius)/(pi/30)) ;//from rad/s  to rpm
    double left_w = int((left_v/wheel_radius)/(pi/30));
    ret = speed(1,right_w*20);
    ret = speed(2,-left_w*20);
	return 0;
}

float info_robot::get_motor_pos(int motor_flag)
{
	float motor_pos;
	uint32 pos_now = place(motor_flag);
	
    if (pos_now > 4e9) 
	{
		motor_pos = -(pow(2,32) - pos_now)*2*wheel_radius*pi/pulse_per_circle;
	}        
    else
	{
 		motor_pos = pos_now*2*wheel_radius*pi/pulse_per_circle;
	}     
	if (motor_flag == left_wheel)
	{
		motor_pos = -motor_pos;
	}
	
	return motor_pos;
}