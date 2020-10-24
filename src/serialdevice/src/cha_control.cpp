#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
using namespace std;
serial::Serial _serial;	
std::string param_port_path_;
int param_baudrate_;
int param_loop_rate_;	

uint8_t commdate[6];
uint16_t linearx;
uint16_t angularz;

void cha_control_callback(const geometry_msgs::Twist& cmd_vel)
{
	
	linearx = (uint16_t)(-cmd_vel.linear.x*1000.0 +1500);
	angularz= (uint16_t)(-cmd_vel.angular.z*1000.0 +1500);
	
	ROS_INFO("[%f,%f,%d,%d]",cmd_vel.linear.x,cmd_vel.angular.z, linearx, angularz);
	commdate[0]=0xAA;
	commdate[1]=linearx/256;
	commdate[2]=linearx%256;
	commdate[3]=angularz/256;
	commdate[4]=angularz%256;
	commdate[5]=commdate[0]+commdate[1]+commdate[2]+commdate[3]+commdate[4];

	_serial.write(commdate, 6);
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "chassis_controler");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, cha_control_callback);
	nh.param<std::string>("chassis_controler/port", param_port_path_, "/dev/car_usb");
	nh.param<int>("chassis_controler/baudrate", param_baudrate_, 9600);
	nh.param<int>("chassis_controler/loop_rate", param_loop_rate_, 20);

	try 
    { 
    //设置串口属性，并打开串口 
        _serial.setPort(param_port_path_); 
        _serial.setBaudrate(param_baudrate_); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        _serial.setTimeout(to); 
        _serial.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(_serial.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port"<<param_port_path_<<" initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

	ros::Rate loop_rate(param_loop_rate_);
	

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
