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

    /* 参数读取 */
    Declare_Parameter();

    /* 初始化消息 */
    Init_Msg();

    /* tf变换对象 */
    this->tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    this->tf_buffer->setCreateTimerInterface(timer_interface);
    this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);
    this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // /* 获取base-imu变换 */
    // this->tf_buffer->waitForTransform(
    //     "base", "imu", tf2::TimePointZero, tf2::Duration(static_cast<int64_t>(1e10)),
    //     [](const tf2_ros::TransformStampedFuture &){}
    // );
    // rclcpp::sleep_for(std::chrono::seconds(1));
    // auto tf_base_imu = this->tf_buffer->lookupTransform("base", "imu", tf2::TimePointZero);
    // tf2::fromMsg(tf_base_imu.transform, this->base_imu);

    /* 创建发布者、订阅者、服务端对象 */
    this->pub_joint_state   = this->create_publisher<sensor_msgs::msg::JointState>
                              ("/joint_states", rclcpp::SensorDataQoS());
    this->pub_imu_base      = this->create_publisher<sensor_msgs::msg::Imu>
                              ("/legged/imu_base", rclcpp::SensorDataQoS());
    this->pub_marker        = this->create_publisher<visualization_msgs::msg::MarkerArray>
                              ("/legged/marker", 10);
    
    this->sub_legged_state  = this->create_subscription<unitree_go::msg::LowState>
                              ("/lowstate", rclcpp::SensorDataQoS(), std::bind(&Class_Legged_Interaction::Legged_State_Callback, this, _1));    
}

/************************************************************************************************************************
 * @brief   足式机器人状态消息订阅回调函数
 ***********************************************************************************************************************/
void Class_Legged_Interaction::Legged_State_Callback(const unitree_go::msg::LowState::ConstSharedPtr legged_state)
{
    //TODO: 可能存在耗时过长问题，待测试频率

    // 记录当前帧时间
    auto frame_now = this->now();

    // 状态读取
    auto imu_state = legged_state->imu_state;
    auto motor_state = legged_state->motor_state;
    auto foot_force = legged_state->foot_force;
    
    // 关节状态消息填充并发布
    this->msg_joint_state.header.stamp = frame_now;
    for (std::size_t i = 0; i < this->msg_joint_state.name.size(); i++)
    {
        this->msg_joint_state.position[i] = motor_state[i].q;
        this->msg_joint_state.velocity[i] = motor_state[i].dq;
        this->msg_joint_state.effort[i] = motor_state[i].tau_est;
    }
    this->pub_joint_state->publish(this->msg_joint_state);

    // IMU 消息填充并发布
    this->msg_imu_base.header.stamp = frame_now;
    this->msg_imu_base.orientation.x = imu_state.quaternion[1];
    this->msg_imu_base.orientation.y = imu_state.quaternion[2];
    this->msg_imu_base.orientation.z = imu_state.quaternion[3];
    this->msg_imu_base.orientation.w = imu_state.quaternion[0];
    this->msg_imu_base.angular_velocity.x = imu_state.gyroscope[0];
    this->msg_imu_base.angular_velocity.y = imu_state.gyroscope[1];
    this->msg_imu_base.angular_velocity.z = imu_state.gyroscope[2];
    this->msg_imu_base.linear_acceleration.x = imu_state.accelerometer[0];
    this->msg_imu_base.linear_acceleration.y = imu_state.accelerometer[1];
    this->msg_imu_base.linear_acceleration.z = imu_state.accelerometer[2];
    this->pub_imu_base->publish(this->msg_imu_base);

    // Marker 消息发布
    this->marker_array.markers.clear();
        // Marker 填充（足端）
    double scale_factor = 1000;
    std::vector<std::string> footforce_frame = {"FR_calflower1", "FL_calflower1", "RR_calflower1", "RL_calflower1"};
    for (std::size_t i = 0; i < footforce_frame.size(); i++)
    {
        this->marker_footforce.header.stamp = frame_now;
        this->marker_footforce.header.frame_id = footforce_frame[i];
        this->marker_footforce.id = i;
        // 设置终点
        geometry_msgs::msg::Point end_point;
        end_point.x = 0.02;
        end_point.y = 0.0;
        end_point.z = foot_force[i] / scale_factor;
        this->marker_footforce.points[1] = end_point;
        // 插入 MarkerArray
        this->marker_array.markers.emplace_back(this->marker_footforce);
    }
    this->pub_marker->publish(this->marker_array);

    // tf变换发布
    //     // 将 base_link->imu 转换为 base_link->base
    // geometry_msgs::msg::TransformStamped tf_baselink_imu;
    // tf_baselink_imu.transform.translation.x = 0.0;
    // tf_baselink_imu.transform.translation.y = 0.0;
    // tf_baselink_imu.transform.translation.z = 0.0;
    // tf_baselink_imu.transform.rotation = this->msg_imu_base.orientation;
    // tf2::Transform baselink_imu;
    // tf2::fromMsg(tf_baselink_imu.transform, baselink_imu);
    // // auto baselink_base = baselink_imu.inverse() * base_imu;
    // auto baselink_base = baselink_imu * base_imu.inverse();
        // 填充消息
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = frame_now;
    transformStamped.header.frame_id = "base_link"; // 父坐标系
    transformStamped.child_frame_id = "base";       // 子坐标系
    // transformStamped.transform = tf2::toMsg(baselink_base);
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation = this->msg_imu_base.orientation;
    this->tf_broadcaster->sendTransform(transformStamped);
}

/************************************************************************************************************************
 * @brief   参数声明函数
 ***********************************************************************************************************************/
void Class_Legged_Interaction::Declare_Parameter()
{
    // 关节名称
    this->joint_names = this->declare_parameter<std::vector<std::string>>("joint_names", {});
    if (this->joint_names.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No joint name found.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Get joint names successfully.");
        for (std::size_t i = 0; i < this->joint_names.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "Joint[%zu]: %s", i, this->joint_names[i].c_str());
        }
    }
}

/************************************************************************************************************************
 * @brief   消息初始化函数
 ***********************************************************************************************************************/
void Class_Legged_Interaction::Init_Msg()
{
    // 关节状态消息
    this->msg_joint_state.name = this->joint_names;
    this->msg_joint_state.position.resize(this->msg_joint_state.name.size(), 0.0);
    this->msg_joint_state.velocity.resize(this->msg_joint_state.name.size(), 0.0);
    this->msg_joint_state.effort.resize(this->msg_joint_state.name.size(), 0.0);

    // IMU 消息
    this->msg_imu_base.header.frame_id = "base_link";

    // Marker 消息
    this->marker_footforce.ns = "footforce";
    this->marker_footforce.type = visualization_msgs::msg::Marker::ARROW;
    this->marker_footforce.action = visualization_msgs::msg::Marker::ADD;
    // 设置起点
    geometry_msgs::msg::Point start_point;
    start_point.x = 0.02;
    start_point.y = 0.0;
    start_point.z = 0.0;
    this->marker_footforce.points.resize(2);
    this->marker_footforce.points[0] = start_point;
    // 设置箭头大小
    this->marker_footforce.scale.x = 0.01;  // 箭杆直径
    this->marker_footforce.scale.y = 0.02;  // 箭头直径
    this->marker_footforce.scale.z = 0.02;  // 箭头长度
    // 设置颜色
    this->marker_footforce.color.r = 0.0;
    this->marker_footforce.color.g = 1.0;
    this->marker_footforce.color.b = 0.0;
    this->marker_footforce.color.a = 1.0;
    // 设置持续时间
    this->marker_footforce.lifetime = rclcpp::Duration(0, 1e7); // 10ms 持续时间
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(legged::Class_Legged_Interaction)
