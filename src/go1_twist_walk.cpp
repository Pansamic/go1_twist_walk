#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"


class Go1TwistWalk : public rclcpp::Node
{
public:
    Go1TwistWalk()
    :Node("go1_twist_walk"), safe_(UNITREE_LEGGED_SDK::LeggedType::Go1), udp_(UNITREE_LEGGED_SDK::HIGHLEVEL, 8090, "192.168.123.161", 8082)
    {
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Go1TwistWalk::twist_callback, this, std::placeholders::_1));
        udp_.InitCmdData(cmd_);
        udp_.GetRecv(state_);
        sdk_send_thread_ = std::thread(&Go1TwistWalk::SdkSend, this);
        sdk_recv_thread_ = std::thread(&Go1TwistWalk::SdkRecv, this);
        sdk_send_thread_.detach();
        sdk_recv_thread_.detach();

        cmd_.mode = 2; // 0:idle, default stand      1:forced stand     2:walk continuously
        cmd_.gaitType = 1;
        cmd_.speedLevel = 0;
        cmd_.footRaiseHeight = 0.1;
        cmd_.bodyHeight = 0;
        cmd_.euler[0] = 0;
        cmd_.euler[1] = 0;
        cmd_.euler[2] = 0;
        cmd_.velocity[0] = 0.0f;
        cmd_.velocity[1] = 0.0f;
        cmd_.yawSpeed = 0.0f;
        cmd_.reserve = 0;
    }
    ~Go1TwistWalk()
    {
        // Stop the robot when the node is destroyed
        // robot->control(0, 0, 0);
    }
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    UNITREE_LEGGED_SDK::Safety safe_;
    UNITREE_LEGGED_SDK::UDP udp_;
    UNITREE_LEGGED_SDK::HighCmd cmd_ = {0};
    UNITREE_LEGGED_SDK::HighState state_ = {0};

    std::thread sdk_send_thread_;
    std::thread sdk_recv_thread_;

    void twist_callback(const geometry_msgs::msg::Twist msg)
    {
        cmd_.velocity[0] = msg.linear.x;
        cmd_.velocity[1] = msg.linear.y;
        cmd_.yawSpeed = msg.angular.z;
        udp_.SetSend(cmd_);
    }

    void SdkSend()
    {
        while(1)
        {
            udp_.Send();
        }
    }

    void SdkRecv()
    {
        while(1)
        {
            udp_.Recv();
        }
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go1TwistWalk>());
    rclcpp::shutdown();
    return 0;
}