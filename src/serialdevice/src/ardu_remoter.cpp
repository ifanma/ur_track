#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 

#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <ur_dashboard_msgs/SetModeAction.h>
#include <ur_dashboard_msgs/RobotMode.h>
#include <serial_dev_msgs/headpan.h>
#include <serial_dev_msgs/Paw.h>
#include "math.h"
#include <string.h>

#define DEADZONE 20
# define rDEADZONE 20

# define max_step_angle 10.0

# define DEG2RAD 3.1415926/180.0
# define RAD2DEG 180.0/3.1415926

serial::Serial ser; //声明串口对象

/* serial */
std::string param_port_path_;
int param_baudrate_;
int param_loop_rate_;

// ===========================================
#define CHANNEL 9
int i = 0;
int j = 0;
uint8_t sum = 0;
int rec_right[CHANNEL] = {0};
int lastrec[CHANNEL] = {0};

int watchdog = 0;

std::vector<double> recf[CHANNEL];
ros::Publisher car_pub; 
ros::Publisher arm_pub;
ros::Publisher head_pub;
ros::Publisher mode_pub;
ros::Publisher paw_pub;

typedef actionlib::SimpleActionClient<ur_dashboard_msgs::SetModeAction> Client;
ur_dashboard_msgs::RobotMode mode_list;
geometry_msgs::Twist cmd_vel;
serial_dev_msgs::headpan cmd_head;
serial_dev_msgs::Paw cmd_paw;
ur_dashboard_msgs::SetModeGoal goal_mode;
ur_dashboard_msgs::SetModeActionGoal msg_goal;


void doneCd(const actionlib::SimpleClientGoalState& state, const ur_dashboard_msgs::SetModeActionResultConstPtr& result)
{
    if (result->result.success){
        ROS_INFO("Mode Set successed");
    }
    else{
        ROS_INFO("Mode Set failed");
    }
}

void activeCd()
{
    ROS_INFO("mode change activated");
}

void feedbackCb(const ur_dashboard_msgs::SetModeActionFeedbackConstPtr& feedback)
{
    ROS_INFO("the Mode right now is: %d", feedback->feedback.current_robot_mode);
}


double fillter(double rec, int index, int times)
{
    double sum = 0.0;
    int i;
    while (recf[index].size() <= times)
    {
        recf[index].push_back(rec);
    }

    while (recf[index].size() > times)
    {
        recf[index].erase(recf[index].begin());

        for (i = 0, sum = 0; i< recf[index].size(); i++)
        {
            sum += recf[index].at(i);
        }

        sum /= (double)(recf[index].size());
    }

    return sum;
}

void MsgSend(char * p)
{
    std_msgs::String cmd_arm;
    cmd_arm.data.append(p);
    arm_pub.publish(cmd_arm);
    cmd_arm.data.clear();  
    ROS_INFO("%s\n", p);
}

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "ardu_remoter_pub"); 
    //声明节点句柄 
    ros::NodeHandle nh; 

    uint8_t rec[2000] = {'\0'}; 
    int intool = 0;
    double armx;
    double army;
    double armz;
    double armRx;
    double armRy;
    double armRz;
    double acc;
    double carx;
    double carz;
    char arm_str[200];

    //发布主题 
    car_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000); 
    arm_pub = nh.advertise<std_msgs::String>("/ur_hardware_interface/script_command", 10);
    head_pub = nh.advertise<serial_dev_msgs::headpan>("/head_pan", 10);
    paw_pub = nh.advertise<serial_dev_msgs::Paw>("/paw_cmd", 10);
    mode_pub = nh.advertise<ur_dashboard_msgs::SetModeActionGoal>("/ur_hardware_interface/set_mode/goal", 10);

	nh.param<std::string>("ardu_remoter_pub/port", param_port_path_, "/dev/remote_USB");
	nh.param<int>("ardu_remoter_pub/baudrate", param_baudrate_, 9600);
	nh.param<int>("ardu_remoter_pub/loop_rate", param_loop_rate_, 20);
	nh.param<double>("ardu_remoter_pub/armx", armx, 0.5);
	nh.param<double>("ardu_remoter_pub/army", army, 0.5);
	nh.param<double>("ardu_remoter_pub/armz", armz, 0.5);
	nh.param<double>("ardu_remoter_pub/armRx", armRx, 0.5);
	nh.param<double>("ardu_remoter_pub/armRy", armRy, 0.5);
	nh.param<double>("ardu_remoter_pub/armRz", armRz, 0.5);
	nh.param<double>("ardu_remoter_pub/acc", acc, 0.5);
	nh.param<double>("ardu_remoter_pub/carx", carx, 1);
	nh.param<double>("ardu_remoter_pub/carz", carz, 1);

    // Client set_mode_client("/ur_hardware_interface/set_mode", true); /* 这里的第一次参数要特别注意,两边代码对于这个名称必须要一致，否则两个节点无法相互通讯。 */
    // /* 等待服务端响应 */
    // ROS_INFO("waiting for server !");
    // set_mode_client.waitForServer();
    // ROS_INFO("action server started !");

    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort(param_port_path_); 
        ser.setBaudrate(param_baudrate_); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port"<<param_port_path_<<" initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    double speedx = 0;
    double speedy = 0;
    double speedz = 0;
    double speedRx = 0;
    double speedRy = 0;
    double speedRz = 0;

    int left_once = 0;
    int first_lock = 1;
    int enonce = 0;
    int leftenonce = 0;
    
    //指定循环的频率 
    ros::Rate loop_rate(param_loop_rate_); 
    while(ros::ok()) 
    { 
        // while(ser.available() < 10);
        if(ser.available()){
            if (first_lock == 0)
            {
                first_lock = 1;
                printf("Armed\n");
            }
            watchdog = 0;

            ser.read(rec,1);
            if (rec[0] == 0xaa)
            {
                ser.read(rec+1,CHANNEL * 2 +1);
                for(j = 0; j < CHANNEL ; ++j){
                    rec_right[j] = ( rec[2*j+1] <<8 ) + rec[2*j+2];
                    sum += rec[2*j+1];
                    sum += rec[2*j+2];
                }
                sum += 0xaa;
                //ROS_INFO("sum=%x;rec[9]=%x\n",sum,rec[9]);
                if (rec[CHANNEL * 2 +1] == sum ){
                    //ROS_INFO("ok\n");
                    for (i = 0; i< CHANNEL; i++)
                    {
                        if (i == 2){
                            rec_right[2] -= 1000;
                        }
                        else{
                            rec_right[i] -= 1500;
                        }
                    }
                    
                    for (i= 0; i <CHANNEL ;i++){
                        if (i == 2){
                            if(rec_right[i] > 1000) rec_right[i] = 1000;
                            if(rec_right[i] < 0) rec_right[i] = 0;
                            if (rec_right[i]<100) rec_right[i] = 0;
                            else rec_right[i] -=100;
                        }
                        else if(i >= 4 && i <= 6){
                            if(rec_right[i] > 500) rec_right[i] = 500;
                            if(rec_right[i] < -500) rec_right[i] = -500;
                            
                            rec_right[i] = (int)(fillter((double)rec_right[i], i, 5));
                            if (abs(rec_right[i] - lastrec[i]) < 4 )
                            {
                                rec_right[i] = lastrec[i];
                            }
                            lastrec[i] = rec_right[i];

                            if (abs(rec_right[i])<rDEADZONE) rec_right[i] = 0;
                            else{
                                if (rec_right[i] > 0) rec_right[i] -= rDEADZONE;
                                if (rec_right[i] < 0) rec_right[i] += rDEADZONE;
                            }
                        }
                        else{
                            if(rec_right[i] > 500) rec_right[i] = 500;
                            if(rec_right[i] < -500) rec_right[i] = -500;
                            if (abs(rec_right[i])<DEADZONE) rec_right[i] = 0;
                            else{
                                if (rec_right[i] > 0) rec_right[i] -= DEADZONE;
                                if (rec_right[i] < 0) rec_right[i] += DEADZONE;
                            }
                        }
                    }

                    // ROS_INFO("1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d 8:%d 9:%d  ",rec_right[0],rec_right[1],rec_right[2],rec_right[3],rec_right[4],rec_right[5],rec_right[6],rec_right[7],rec_right[8]);
                    if (rec_right[5] > 400){
                        if (abs(rec_right[7]) < DEADZONE)       // 右手拨杆中位, car
                        {
                            cmd_vel.linear.x  = float( rec_right[2] * rec_right[1] ) / 450000.0 * carx;
                            cmd_vel.linear.y  = 0.0; //float( rec_right[2] ) * float( rec_right[0] ) / 450000.0 * MAX_y ;
                            cmd_vel.linear.z  = 0;
                            cmd_vel.angular.x = 0;
                            cmd_vel.angular.y = 0;
                            cmd_vel.angular.z = float( rec_right[2] * rec_right[3] ) / 450000.0 * carz ;

                            car_pub.publish(cmd_vel);

                            // tuizi 
                            // if (abs(rec_right[0] ) <DEADZONE)
                            // {
                            //     if(rec_right[8] > 200){
                            //         if (rec_right[1] > 400){
                            //             if (enonce == 0){
                            //                 enonce = 1;
                                            
                            //             }
                            //         }
                            //         else if (rec_right[1] < -400)
                            //         {
                            //             if (enonce == 0){
                            //                 enonce = 1;
                            //             }
                            //         }
                            //         else if (abs(rec_right[1]) < DEADZONE)
                            //         {
                            //             enonce = 0;
                            //         }
                            //     }
                            // }
                        }
                        else if (rec_right[7] > 400)        // 右手拨杆向前, arm
                        {
                            if (rec_right[2] > 600 && rec_right[2] < 800){
                                speedx = 0;
                                speedy = 0;
                                speedz = 0;
                                speedRx = float( rec_right[1] ) / 500.0 * armRx;
                                speedRy = float( rec_right[3] ) / 500.0 * armRy;
                                speedRz = float( rec_right[0] ) / 500.0 * armRz;
                                if(rec_right[8] > 200)
                                {
                                    left_once = 0;
                                    sprintf(arm_str,"speedl([%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],a=%.3f,t=0.5)\n", speedx, speedy, speedz, speedRx, speedRy, speedRz, acc);
                                    MsgSend(arm_str);
                                }
                            }
                            else if (rec_right[2] < 400){
                                speedRx = 0;
                                speedRy = 0;
                                speedRz = 0;
                                speedx = float( rec_right[1] ) / 500.0 * armx;
                                speedy = float( rec_right[3] ) / 500.0 * army;
                                speedz = float( rec_right[0] ) / 500.0 * armz;
                                if(rec_right[8] > 200)
                                {
                                    left_once = 0;
                                    sprintf(arm_str,"speedl([%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],a=%.3f,t=0.5)\n", speedx, speedy, speedz, speedRx, speedRy, speedRz, acc);
                                    MsgSend(arm_str);
                                }
                            }
                            else if (rec_right[2] > 880){
                                if(rec_right[8] > 200)
                                {
                                    if (left_once == 1)
                                    {
                                        left_once = 0;
                                        sprintf(arm_str,"movej([-1.57, -1.57, -1.57, 0, 1.57, 0],a=%.3f,v=0.5,t=0,r=0)\n", acc);
                                        MsgSend(arm_str);
                                    }
                                }
                            }

                            if (rec_right[8] < 150) {
                                if (left_once == 0)
                                {
                                    left_once = 1;
                                    sprintf(arm_str,"stopl(10)");
                                    MsgSend(arm_str);
                                }

                                if (rec_right[0] > 400)
                                {
                                    if (rec_right[1] > 400){
                                        if (leftenonce == 0){
                                            leftenonce = 1;
                                            msg_goal.goal.target_robot_mode = mode_list.RUNNING;
                                            mode_pub.publish(msg_goal);
                                            // goal_mode.target_robot_mode = mode_list.RUNNING;
                                            // set_mode_client.sendGoal(goal_mode);
                                        }
                                    }
                                    else if (rec_right[1] < -400)
                                    {
                                        if (leftenonce == 0){
                                            leftenonce = 1;
                                            msg_goal.goal.target_robot_mode = mode_list.POWER_OFF;
                                            mode_pub.publish(msg_goal);
                                            // goal_mode.target_robot_mode = mode_list.POWER_OFF;
                                            // set_mode_client.sendGoal(goal_mode);
                                        }
                                    }
                                    else if (abs(rec_right[1]) < DEADZONE)
                                    {
                                        leftenonce = 0;
                                    }
                                }
                            }
                        }
                        else if (rec_right[7] < -400)       // 右手拨杆向后, head
                        {
                            cmd_head.pitch = float( rec_right[4] ) / 480.0 * 90.0 + 90.0;
                            cmd_head.yaw = float( rec_right[6] ) / 480.0 * 90.0 + 90.0;
                            head_pub.publish(cmd_head);

                            cmd_paw.rightPawPosition = 90.0 - float( rec_right[2] ) / 900.0 * 90.0 ;
                            paw_pub.publish(cmd_paw);

                        }
                    }
                }
                sum = 0;
                ser.flushInput(); 
            }
        } 
        else{
            watchdog++;
            if (watchdog > param_loop_rate_ * 3){  // 3秒检测不到则认为已经加锁
                watchdog = 0;
                if (first_lock == 1)
                {
                    first_lock = 0;
                    printf("DisArmed\n");
                }
            }
        }    
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } //
    ser.close();
}
