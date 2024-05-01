#include <chrono>
#include <iostream>
#include <functional>

#include "clobot_platform_libs/geometry/orientation.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2_exception.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using namespace std::chrono_literals;


class TFSubNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_tf_signal_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;


public:
    TFSubNode()
    :Node("tf_sub_node")
    {
        sub_tf_signal_ = this->create_subscription<std_msgs::msg::UInt8>(
            "tf_signal", 
            10, 
            std::bind(&TFSubNode::on_subscribed_tf_signal, this, std::placeholders::_1));


        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }


private:
    void on_subscribed_tf_signal(const std_msgs::msg::UInt8::SharedPtr tf_signal_msg)
    {
        geometry_msgs::msg::PoseStamped pose_in_map;
        geometry_msgs::msg::PoseStamped pose_in_base_link;
        geometry_msgs::msg::PoseStamped pose_in_POI_frame;


        pose_in_map.header.stamp = this->get_clock()->now();
        pose_in_map.header.frame_id = "map";


        pose_in_base_link.header.stamp = this->get_clock()->now();
        pose_in_base_link.header.frame_id = "base_link";

        pose_in_base_link.pose.position.x = 0;
        pose_in_base_link.pose.position.y = 0;
        pose_in_base_link.pose.position.z = 0;

        pose_in_base_link.pose.orientation.x = 0;
        pose_in_base_link.pose.orientation.y = 0;
        pose_in_base_link.pose.orientation.z = 0;
        pose_in_base_link.pose.orientation.w = 1;


        pose_in_base_link.header.stamp = this->get_clock()->now();
        pose_in_base_link.header.frame_id = "POI_frame";

        pose_in_base_link.pose.position.x = 0;
        pose_in_base_link.pose.position.y = 0;
        pose_in_base_link.pose.position.z = 0;

        pose_in_base_link.pose.orientation.x = 0;
        pose_in_base_link.pose.orientation.y = 0;
        pose_in_base_link.pose.orientation.z = 0;
        pose_in_base_link.pose.orientation.w = 1;


        try
        {
            geometry_msgs::msg::TransformStamped tf_base_link_to_map = tf_buffer_->lookupTransform(
                "map",
                "base_link",
                tf2::TimePointZero,
                tf2::Duration(5000000)); // 5 ms
            
            tf2::doTransform(pose_in_base_link, pose_in_map, tf_base_link_to_map);
        }
        catch (const tf2::TransformException& ex)
        {
            std::cout << "base_link <-> map can't lookupTransform()" << std::endl;

            return;
        }

        auto euler = clobot_platform_libs::geometry::Quaternion::ToEulerAngles(
            pose_in_base_link.pose.orientation.x,
            pose_in_base_link.pose.orientation.y,
            pose_in_base_link.pose.orientation.z,
            pose_in_base_link.pose.orientation.w
        );


        std::cout << "pose in base_link\t" << "x: " << pose_in_base_link.pose.position.x << ", y: " << pose_in_base_link.pose.position.y
                    << ", yaw: " << euler.Yaw << std::endl;


        euler = clobot_platform_libs::geometry::Quaternion::ToEulerAngles(
            pose_in_map.pose.orientation.x,
            pose_in_map.pose.orientation.y,
            pose_in_map.pose.orientation.z,
            pose_in_map.pose.orientation.w
        );


        std::cout << "=> pose in map\t" << "x: " << pose_in_map.pose.position.x << ", y: " << pose_in_map.pose.position.y
                    << ", yaw: " << euler.Yaw << std::endl;


    }
};