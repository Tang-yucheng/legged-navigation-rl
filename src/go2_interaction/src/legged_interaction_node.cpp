/**
 * @file    legged_interaction_node.cpp
 * @brief   足式机器人交互节点
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2025-3-18
 * @version v1.0
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "legged_interaction_node.hpp"

namespace legged
{

/************************************************************************************************************************
 * @brief   足式机器人交互节点类构造函数
 ***********************************************************************************************************************/
Class_Legged_Interaction::Class_Legged_Interaction(const rclcpp::NodeOptions & options)
: Node("legged_interaction_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting legged interaction node!");

    /* 配置参数读取 */
    this->joint_names = this->declare_parameter<std::vector<std::string>>("joint_names", {});
    if (this->joint_names.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No joint find, return.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Get joint names successfully.");
        for (std::size_t i = 0; i < this->joint_names.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "Joint[%zu]: %s", i, this->joint_names[i].c_str());
        }
    }

    /* 初始化 Marker 消息 */
    Init_Msg();
    Init_Marker_Msg();

    /* 创建发布者、订阅者、服务端对象 */
    this->pub_joint_state   = this->create_publisher<sensor_msgs::msg::JointState>
                              ("/joint_states", rclcpp::SensorDataQoS());
    // this->pub_marker        = this->create_publisher<visualization_msgs::msg::MarkerArray>
    //                           ("/legged/marker", 10);
    
    this->sub_legged_state  = this->create_subscription<unitree_go::msg::LowState>
                              ("/lowstate", rclcpp::SensorDataQoS(), std::bind(&Class_Legged_Interaction::Legged_State_Callback, this, _1));    
}

/************************************************************************************************************************
 * @brief   足式机器人状态消息订阅回调函数
 ***********************************************************************************************************************/
void Class_Legged_Interaction::Legged_State_Callback(const unitree_go::msg::LowState::ConstSharedPtr legged_state)
{
    this->msg_joint_state.header.stamp = this->now();
    
    // 模拟关节运动（正弦波变化）
    double time = this->now().seconds();
    this->msg_joint_state.position[0] = sin(time);
    this->msg_joint_state.position[1] = cos(time);
    this->msg_joint_state.position[2] = sin(2 * time);

    this->pub_joint_state->publish(this->msg_joint_state);

    
}

/************************************************************************************************************************
 * @brief   消息初始化函数
 ***********************************************************************************************************************/
void Class_Legged_Interaction::Init_Msg()
{
    // 初始化关节名称
    this->msg_joint_state.name = this->joint_names;
    this->msg_joint_state.position.resize(this->msg_joint_state.name.size(), 0.0);
    this->msg_joint_state.velocity.resize(this->msg_joint_state.name.size(), 0.0);
    this->msg_joint_state.effort.resize(this->msg_joint_state.name.size(), 0.0);
}

/************************************************************************************************************************
 * @brief   Marker消息初始化函数
 ***********************************************************************************************************************/
void Class_Legged_Interaction::Init_Marker_Msg()
{

}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(legged::Class_Legged_Interaction)
