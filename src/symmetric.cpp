#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>
#include <iostream>
#include <algorithm>

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
    double Kp;

    pnh.param<double>("rate", rate, 100.0);
    pnh.param<std::string>("port_name", port_name, "/dev/ttyUSB0");
    pnh.param<int>("baudrate", baudrate, 115200);
    pnh.param<double>("current_limit", current_limit, 50.0);
    pnh.param<double>("Kp", Kp, 0.1);

    ros::Rate loop_rate(rate);

    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name, baudrate);
    dynamixel_wrapper::dynamixel_wrapper motor0(0, dxl_base, dynamixel_wrapper::XL330, 0);
    dynamixel_wrapper::dynamixel_wrapper motor1(1, dxl_base, dynamixel_wrapper::XL330, 0);

    motor0.setTorqueEnable(true);
    motor1.setTorqueEnable(true);
    motor0.setCurrentLimit(current_limit);
    motor1.setCurrentLimit(current_limit);


    while (nh.ok())
    {
        //get state
        double motor0_th = motor0.getPresentPosition();
        double motor1_th = motor1.getPresentPosition();

        double motor0_dth = motor0.getPresentVelocity();
        double motor1_dth = motor1.getPresentVelocity();

        double motor0_current = motor0.getPresentCurrent();
        double motor1_current = motor1.getPresentCurrent();

        double motor0_setCurrent = Kp*(motor1_th - motor0_th);
        double motor1_setCurrent = -Kp*(motor1_th - motor0_th);

        motor0_setCurrent = std::clamp(motor0_setCurrent, -current_limit, current_limit);
        motor1_setCurrent = std::clamp(motor1_setCurrent, -current_limit, current_limit);

        motor0.setGoalCurrent(motor0_setCurrent);
        motor1.setGoalCurrent(motor1_setCurrent);

        std::cout<<"motor0 pos:"<<motor0_th<<" vel:"<<motor0_dth<<" current:"<<motor0_current<<std::endl;
        std::cout<<"motor1 pos:"<<motor1_th<<" vel:"<<motor1_dth<<" current:"<<motor1_current<<std::endl;
        std::cout<<"----------------------------------"<<std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}