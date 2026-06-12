#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/idl/go2/LowState_.hpp"
#include <unitree/robot/b2/sport/sport_client.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

// 全局变量用于存储速度信息
double vx = 0.0, vy = 0.0, yaw = 0.0;
unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;//b2底层信息DDS订阅
unitree_go::msg::dds_::LowState_ low_state{};  // default init
ros::Publisher imu_raw_pub;

// cmd_vel 回调函数
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    yaw = msg->angular.z;
}
//b2底层数据DDS回调
void LowStateMessageHandler(const void* message)
{
    sensor_msgs::Imu imu_raw;
    low_state = *(unitree_go::msg::dds_::LowState_*)message;

    imu_raw.header.stamp = ros::Time::now();
    imu_raw.header.frame_id = "imu_link";
    //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
    imu_raw.orientation.x = low_state.imu_state().quaternion()[0];
    imu_raw.orientation.y = low_state.imu_state().quaternion()[1];
    imu_raw.orientation.z = low_state.imu_state().quaternion()[2];
    imu_raw.orientation.w = low_state.imu_state().quaternion()[3];
    //线加速度
    imu_raw.linear_acceleration.x = low_state.imu_state().accelerometer()[0]; 
    imu_raw.linear_acceleration.y = low_state.imu_state().accelerometer()[1];
    imu_raw.linear_acceleration.z = low_state.imu_state().accelerometer()[2];
	//角速度
    imu_raw.angular_velocity.x = low_state.imu_state().gyroscope()[0];
    imu_raw.angular_velocity.y = low_state.imu_state().gyroscope()[1];
    imu_raw.angular_velocity.z = low_state.imu_state().gyroscope()[2];
 
    imu_raw_pub.publish(imu_raw); //imu_raw_pub 节点发布消息至imu_data topic
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    /*
     * Initilaize ChannelFactory
     */
    unitree::robot::ChannelFactory::Instance()->Init(0);
    // unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    // 初始化 ROS 节点
    ros::init(argc, argv, "unitree_move_node");
    ros::NodeHandle nh;

    // 订阅 cmd_vel 话题
    ros::Subscriber sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);
    imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);


    lowstate_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
    lowstate_subscriber->InitChannel(std::bind(&LowStateMessageHandler, std::placeholders::_1), 1);

    unitree::robot::b2::SportClient sc;
    sc.SetTimeout(5.0f);
    sc.Init();


    // 设置循环频率
    ros::Rate loop_rate(100); // 100 Hz

    //Test Api
    while (ros::ok())
    {
        // 调用 Move 函数，传入从 cmd_vel 获取的速度信息
        // int32_t ret = sc.Move(vx, vy, yaw);
        // std::cout << "vx="<<vx<<std::endl;
        // std::cout << "Call Move ret:" << ret << std::endl;

        // 处理回调函数
        ros::spinOnce();

        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}