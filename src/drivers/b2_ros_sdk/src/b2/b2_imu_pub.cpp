#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/idl/go2/LowState_.hpp"
#include <unitree/robot/b2/sport/sport_client.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>


class B2_Node: public ros::NodeHandle
{
private:
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber; //b2底层信息DDS订阅
    unitree_go::msg::dds_::LowState_ low_state{}; // 底层数据变量
    ros::Publisher imu_raw_pub_; //IMU话题发布
    // ros::Subscriber cmd_vel_sub;

public:
    B2_Node()
    {
        // 订阅 cmd_vel 话题
        // cmd_vel_sub = this->subscribe("cmd_vel", 10, cmdVelCallback);
        lowstate_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate")); //DDS订阅底层数据
        lowstate_subscriber->InitChannel(std::bind(&B2_Node::LowStateMessageHandler, this, std::placeholders::_1), 1); //DDS回调设置
        imu_raw_pub_ = this->advertise<sensor_msgs::Imu>("imu_data", 10); //ROS imu话题
        
        // sc.SetTimeout(5.0f);
        // sc.Init();
    }
    ~B2_Node(){}

private:
    //b2底层数据DDS回调
    void LowStateMessageHandler(const void* message)
    {
        sensor_msgs::Imu imu_raw;
        this->low_state = *(unitree_go::msg::dds_::LowState_*)message;

        imu_raw.header.stamp = ros::Time::now();
        imu_raw.header.frame_id = "b2_imu";
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
    
        imu_raw_pub_.publish(imu_raw); //imu_raw_pub 节点发布消息至imu_data topic
    }
};

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
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    // 初始化 ROS 节点
    ros::init(argc, argv, "b2_imu_pub");
    B2_Node b2_node;
    ros::spin();
    return 0;
}