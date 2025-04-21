#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

using namespace std::chrono_literals;

std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

tf2::Transform tf_lidar_base;
geometry_msgs::msg::TransformStamped msg_lidar_base;
geometry_msgs::msg::TransformStamped msg_odom_slam;

void slamCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 填充 slam 里程计原始变换
    msg_odom_slam.header.stamp = msg->header.stamp;
    msg_odom_slam.header.frame_id = msg->header.frame_id;
    msg_odom_slam.child_frame_id = msg->child_frame_id;
    msg_odom_slam.transform.translation.x = msg->pose.pose.position.x;
    msg_odom_slam.transform.translation.y = msg->pose.pose.position.y;
    msg_odom_slam.transform.translation.z = msg->pose.pose.position.z;
    msg_odom_slam.transform.rotation = msg->pose.pose.orientation;
    
    // 填充并发布 camera_init 在 world 下变换
    geometry_msgs::msg::TransformStamped msg_world_init;
    msg_world_init.header.stamp = msg->header.stamp;
    msg_world_init.header.frame_id = "world";
    msg_world_init.child_frame_id = "camera_init";
    msg_world_init.transform.translation.x = 0.0;
    msg_world_init.transform.translation.y = 0.0;
    msg_world_init.transform.translation.z = 0.0;
    msg_world_init.transform.rotation.x = 0.0;
    msg_world_init.transform.rotation.y = 0.0;
    msg_world_init.transform.rotation.z = 0.0;
    msg_world_init.transform.rotation.w = 1.0;
    tf_broadcaster->sendTransform(msg_world_init);

    // 计算 base 在 world下的变换
    tf2::Transform tf_world_init;
    tf2::fromMsg(msg_world_init.transform, tf_world_init);
    tf2::Transform tf_slam;
    tf2::fromMsg(msg_odom_slam.transform, tf_slam);
    tf2::Transform tf_world_base;
    tf_world_base = tf_world_init * tf_slam * tf_lidar_base;

    // 填充并发布 base 在 world 下变换
    geometry_msgs::msg::TransformStamped msg_world_base;
    msg_world_base.header.stamp = msg->header.stamp;
    msg_world_base.header.frame_id = "world";
    msg_world_base.child_frame_id = "base";
    msg_world_base.transform = tf2::toMsg(tf_world_base);
    tf_broadcaster->sendTransform(msg_world_base);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("slam_tf_publisher_node");

    // tf变换对象初始化
    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*node);

    // 等待tf初始化完成
    rclcpp::sleep_for(1s);

    // 获取 base 在 lidar 下的变化
    try 
    {
        msg_lidar_base = tf_buffer->lookupTransform("lidar_mid360", "base", rclcpp::Time(0), 3s);
        tf2::fromMsg(msg_lidar_base.transform, tf_lidar_base);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(node->get_logger(), "TF2 Exception: %s", ex.what());
    }

    // 订阅slam里程计
    auto slam_sub = node->create_subscription<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 1000, slamCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
