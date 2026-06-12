/**
 * 底层速度控制
 */


#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/idl/go2/LowState_.hpp"
#include "unitree/idl/go2/SportModeState_.hpp"
#include <unitree/robot/b2/sport/sport_client.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <iomanip>

using namespace unitree::robot::b2;

class B2_Node: public ros::NodeHandle
{
private:
    unitree::robot::b2::MotionSwitcherClient msc;//运控状态消息
    unitree::robot::b2::SportClient sc;//运控状态消息
  
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sportstate_subscriber_; //b2上层信息DDS订阅

    ros::Subscriber cmd_vel_sub_;


    unitree_go::msg::dds_::SportModeState_ high_state_{}; // 上层数据变量

    ros::Time HighStateTime;
    // ros::Time CmdVelTime;
    int32_t motionStatus;//运控服务状态 0无运控 1有运控
    std::string client_mode;//运控服务名 ai normal

    std::string ROBOT_FORM = "0";//机器人模式

public:
    B2_Node()
    {
        ros::Time::init();
        /*init MotionSwitcherClient*/
        msc.SetTimeout(10.0f); 
        msc.Init();
        while(msc_init()) if(ros::ok()) sleep(1); else break;
    
        sportstate_subscriber_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>("rt/sportmodestate")); //DDS订阅上层数据
        sportstate_subscriber_->InitChannel(std::bind(&B2_Node::HighStateMessageHandler, this, std::placeholders::_1), 1); //DDS回调设置
        cmd_vel_sub_ = this->subscribe<geometry_msgs::Twist>("cmd_vel", 10, std::bind(&B2_Node::cmdVelCallback, this, std::placeholders::_1)); // 订阅 cmd_vel 话题
        
        sc.SetTimeout(5.0f);
        sc.Init();
        HighStateTime = ros::Time::now();

    }
    ~B2_Node(){}

private:

    void HighStateMessageHandler(const void* message)
    {
        high_state_ = *(unitree_go::msg::dds_::SportModeState_ *)message;
        ros::Duration diff = ros::Time::now()-HighStateTime;
        if(diff.toSec()<1.0) return;
        // printf("\033[1;92malready get high state msg!\n\033[0m");
        // printf("\033[1;92m%f\n\033[0m",diff.toSec());
        std::cout<<"ACTION MODE: "<<(int)high_state_.mode()<<std::endl;
        std::cout<<"GAIT TYPE: "<<(int)high_state_.gait_type()<<std::endl;
        std::cout<<std::fixed<<std::setprecision(2)<<"VELOCITY: x="<<high_state_.velocity()[0]<<" y="<<high_state_.velocity()[1]<<" z="<<high_state_.velocity()[2]<<std::endl;
        std::cout<<"YAW SPEED: "<<high_state_.yaw_speed()<<std::endl;
        HighStateTime = ros::Time::now();
        
    }
    

    int32_t msc_init(void)
    {
        std::string a;
        int32_t ret = msc.CheckMode(a, client_mode);
        if (ret == 0) {
            std::cout << "\033[1;32CheckMode succeeded.\033[0m" << std::endl;
        } else {
            std::cout << "\033[1;31mCheckMode failed. Error code: " << ret <<"\033[0m"<< std::endl;
            return 1;//识别错误
        }
        if(client_mode.empty())
        {
            std::cout << "\033[1;33The motion control-related service is deactivated.\033[0m" << std::endl;
            motionStatus = 0;
        }
        else
        {
            std::cout << "\033[1;32mService: "<<client_mode<< " is activate\033[0m" << std::endl;
            motionStatus = 1;
        }
        return 0;
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& message)
    {
        printf("\033[1;92malready get cmd_vel msg!\n\033[0m");
        uint8_t ifbalabce = high_state_.mode();
        float vx,vy,vyaw;
        vx   = message->linear.x;
        vy   = message->linear.y;
        vyaw = message->angular.z;
        if((ifbalabce!=1)) {sc.Move(0,0,0); std::cout << "\033[1;31mMODE IS WRONG, CAN'T MOVE!\033[0m"<< std::endl;}
        else if((fabs(vx)>0.6)||(fabs(vy)>0.4)||(fabs(vyaw)>0.8)){sc.Move(0,0,0); std::cout << "\033[1;31mINPUT IS OUT OF RANGE!\033[0m"<< std::endl;}
        else
        {
            sc.Move(vx,vy,vyaw);
            std::cout << "\033[1;31mREADY TO MOVE!\033[0m"<< std::endl;
            std::cout <<vx<< std::endl;
            std::cout <<vy<< std::endl;
            std::cout <<vyaw<< std::endl;
        }
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
    ros::init(argc, argv, "velocity_control");
    B2_Node b2_node;
    ros::spin();
    return 0;
}