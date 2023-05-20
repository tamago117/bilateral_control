#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>
#include <iostream>
#include <algorithm>
#include <vector>

double motor0_vr_position = 0.0;
void motor0_callback(const std_msgs::Float64ConstPtr &msg)
{
    motor0_vr_position = msg->data;
}

double motor1_vr_position = 0.0;
void motor1_callback(const std_msgs::Float64ConstPtr &msg)
{
    motor1_vr_position = msg->data;
}

double motor2_vr_position = 0.0;
void motor2_callback(const std_msgs::Float64ConstPtr &msg)
{
    motor2_vr_position = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "symmetric");
    //制御周期10Hz

    //param setting
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double rate;
    std::string port_name;
    int baudrate;
    double current_limit;
    std::vector<double> Kp{3};
    std::vector<double> Kd{3};

    pnh.param<double>("rate", rate, 1000.0);
    pnh.param<std::string>("port_name", port_name, "/dev/ttyUSB0");
    pnh.param<int>("baudrate", baudrate, 115200);
    pnh.param<double>("current_limit", current_limit, 80.0);
    
    for(int i=0; i<3; i++){
        std::string param_name = "Kp" + std::to_string(i);
        pnh.param<double>(param_name, Kp[i], 1);
        std::string param_name2 = "Kd" + std::to_string(i);
        pnh.param<double>(param_name2, Kd[i], 1);
    }

    //publisher
    ros::Publisher motor0_pub = nh.advertise<std_msgs::Float64>("motor0", 1);
    ros::Publisher motor1_pub = nh.advertise<std_msgs::Float64>("motor1", 1);
    ros::Publisher motor2_pub = nh.advertise<std_msgs::Float64>("motor2", 1);

    //subscriber
    ros::Subscriber motor0_sub = nh.subscribe("motor0_vr", 1, motor0_callback);
    ros::Subscriber motor1_sub = nh.subscribe("motor1_vr", 1, motor1_callback);
    ros::Subscriber motor2_sub = nh.subscribe("motor2_vr", 1, motor2_callback);

    ros::Rate loop_rate(rate);

    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name, baudrate);
    dynamixel_wrapper::dynamixel_wrapper motor0(0, dxl_base, dynamixel_wrapper::XL330, 0);
    dynamixel_wrapper::dynamixel_wrapper motor1(1, dxl_base, dynamixel_wrapper::XL330, 0);
    dynamixel_wrapper::dynamixel_wrapper motor2(2, dxl_base, dynamixel_wrapper::XL330, 0);

    motor0.setTorqueEnable(true);
    motor1.setTorqueEnable(true);
    motor2.setTorqueEnable(true);
    motor0.setCurrentLimit(current_limit);
    motor1.setCurrentLimit(current_limit);
    motor2.setCurrentLimit(current_limit);


    while (nh.ok())
    {
        //get state
        double motor0_th = motor0.getPresentPosition();
        double motor1_th = motor1.getPresentPosition();
        double motor2_th = motor2.getPresentPosition();

        double motor0_dth = motor0.getPresentVelocity();
        double motor1_dth = motor1.getPresentVelocity();
        double motor2_dth = motor2.getPresentVelocity();

        double motor0_setCurrent = -Kp[0]*(motor0_th - motor0_vr_position) - Kd[0]*motor0_dth;
        double motor1_setCurrent = -Kp[1]*(motor1_th - motor1_vr_position) - Kd[1]*motor1_dth;
        double motor2_setCurrent = -Kp[2]*(motor2_th - motor2_vr_position) - Kd[2]*motor2_dth;

        motor0_setCurrent = std::clamp(motor0_setCurrent, -current_limit, current_limit);
        motor1_setCurrent = std::clamp(motor1_setCurrent, -current_limit, current_limit);
        motor2_setCurrent = std::clamp(motor2_setCurrent, -current_limit, current_limit);

        motor0.setGoalCurrent(motor0_setCurrent);
        motor1.setGoalCurrent(motor1_setCurrent);
        motor2.setGoalCurrent(motor2_setCurrent);

        //publish
        std_msgs::Float64 motor0_msg;
        std_msgs::Float64 motor1_msg;
        std_msgs::Float64 motor2_msg;

        motor0_msg.data = motor0_th;
        motor1_msg.data = motor1_th;
        motor2_msg.data = motor2_th;

        motor0_pub.publish(motor0_msg);
        motor1_pub.publish(motor1_msg);
        motor2_pub.publish(motor2_msg);

        std::cout<<"motor0 pos:"<<motor0_th<<" vel:"<<motor0_dth<<" current:"<<motor0_setCurrent<<std::endl;
        std::cout<<"motor1 pos:"<<motor1_th<<" vel:"<<motor1_dth<<" current:"<<motor1_setCurrent<<std::endl;
        std::cout<<"motor2 pos:"<<motor2_th<<" vel:"<<motor2_dth<<" current:"<<motor2_setCurrent<<std::endl;
        std::cout<<"----------------------------------"<<std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}