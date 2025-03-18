/**
 * @file    legged_interaction_node.hpp
 * @brief   足式机器人交互节点
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2025-3-18
 * @version v1.0
 */

#ifndef __LEGGED_INTERACTION_NODE_HPP
#define __LEGGED_INTERACTION_NODE_HPP

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
/* ros库 */
#include <rclcpp/rclcpp.hpp>

/* msg头文件 */
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/motor_state.hpp"

/* std库 */

/* user库 */

using std::placeholders::_1;

namespace legged
{

/* 类定义 --------------------------------------------------------------------------------------------------------------*/
/**
 * @brief  足式机器人交互节点节点类
 */
class Class_Legged_Interaction : public rclcpp::Node
{
public:
    Class_Legged_Interaction(const rclcpp::NodeOptions & options);
private:
    /* ---------------------------------------- 函数 --------------------------------------------- */

    // 初始化函数
    void Init_Msg();
    void Init_Marker_Msg();

    // 回调函数
    void Legged_State_Callback(const unitree_go::msg::LowState::ConstSharedPtr legged_state);

    // 核心功能实现函数

    // debug 相关函数
    
    /* ---------------------------------------- 常量 --------------------------------------------- */

    // 发布者
    rclcpp::Publisher                                       /*!< 关节状态消息发布对象 */
    <sensor_msgs::msg::JointState>::SharedPtr pub_joint_state;
    
    // 订阅者
    rclcpp::Subscription                                    /*!< 足式机器人状态消息订阅对象 */
    <unitree_go::msg::LowState>::SharedPtr sub_legged_state;

    // 服务端

    /* ---------------------------------------- 变量 --------------------------------------------- */

    // 核心变量
    std::vector<std::string> joint_names;                   /*!< 关节名称 */
    sensor_msgs::msg::JointState msg_joint_state;           /*!< 关节状态消息 */

    // 时间相关

    // Marker 相关
    visualization_msgs::msg::Marker marker_armor;           /*!< Marker 消息（识别到装甲板） */

    // debug 相关
};

}  // namespace legged

#endif	/* legged_interaction_node.hpp */
