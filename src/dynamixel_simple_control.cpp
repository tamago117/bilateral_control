#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>

double id0_setCurrent = 0.0;
void ampCallback0(const std_msgs::Float64& msg)
{
    id0_setCurrent = msg.data;
}

double id1_setCurrent = 0.0;
void ampCallback1(const std_msgs::Float64& msg)
{
    id1_setCurrent = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_simple_control");
    ros::NodeHandle n;
    double rate=10.0;
    //制御周期10Hz
    ros::Rate loop_rate(rate);

    //param setting
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");
    
    ros::Subscriber amp_sub0 = nh.subscribe("id0/set_current", 1, ampCallback0);
    ros::Subscriber amp_sub1 = nh.subscribe("id1/set_current", 1, ampCallback1);

    std::string port_name("/dev/ttyUSB0");
    int baudrate=115200;

    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name,baudrate);
    dynamixel_wrapper::dynamixel_wrapper motor0(0, dxl_base, dynamixel_wrapper::XL330, 0);
    dynamixel_wrapper::dynamixel_wrapper motor1(1, dxl_base, dynamixel_wrapper::XL330, 0);

    motor0.setTorqueEnable(true);
    motor1.setTorqueEnable(true);
    motor0.setCurrentLimit(40.0);
    motor1.setCurrentLimit(40.0);


    while (n.ok())
    {
        motor0.setGoalCurrent(id0_setCurrent);
        motor1.setGoalCurrent(id1_setCurrent);

        double motor0_th = motor0.getPresentPosition();
        double motor1_th = motor1.getPresentPosition();

        double motor0_dth = motor0.getPresentVelocity();
        double motor1_dth = motor1.getPresentVelocity();

        double motor0_current = motor0.getPresentCurrent();
        double motor1_current = motor1.getPresentCurrent();

        std::cout<<"motor0 pos:"<<motor0_th<<" vel:"<<motor0_dth<<" current:"<<motor0_current<<std::endl;
        std::cout<<"motor1 pos:"<<motor1_th<<" vel:"<<motor1_dth<<" current:"<<motor1_current<<std::endl;
        std::cout<<"----------------------------------"<<std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}