#include <chrono>
#include <iostream>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


using namespace std::chrono_literals;


/*
1 sec 주기로 'tf_signal' Topic 및 frame 간 tf 관계 발행
*/
class TFPubNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_tf_signal_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_; // frame 간 static tf 발행
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // frame 간 동적 tf 발행

    rclcpp::TimerBase::SharedPtr tf_pub_timer_;


public:
    TFPubNode()
    : Node("tf_pub_node")
    {
        pub_tf_signal_ = this->create_publisher<std_msgs::msg::UInt8>("tf_signal", 10);

        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        tf_pub_timer_ = this->create_wall_timer(1s, std::bind(&TFPubNode::tf_pub_timer_func, this));
    }


private:
    void tf_pub_timer_func()
    {
        geometry_msgs::msg::TransformStamped tf_map_to_base_link;
        geometry_msgs::msg::TransformStamped tf_base_link_to_POI_frame;

        std_msgs::msg::UInt8 tf_signal_msg;

/*
step 1.
'TransformStamped' 타입의 msg를 생성하고,
해당 msg에 두 개의 frame간 평행이동 -> 회전변환 관계를 정의한다.
'*.header.frame_id' 에 들어가는 frame이 부모 frame이 되고,
'*.child_frame_id' 에 들어가는 frame이 자식 frame이 된다.
*/
        tf_map_to_base_link.header.stamp = this->get_clock()->now();
        tf_map_to_base_link.header.frame_id = "map";
        tf_map_to_base_link.child_frame_id = "base_link";

        tf_map_to_base_link.transform.translation.x = 1.0;
        tf_map_to_base_link.transform.translation.y = 1.0;
        tf_map_to_base_link.transform.translation.z = 0.0;

        // 45 deg
        tf_map_to_base_link.transform.rotation.x = 0.0;
        tf_map_to_base_link.transform.rotation.y = 0.0;
        tf_map_to_base_link.transform.rotation.z = 0.3826834;
        tf_map_to_base_link.transform.rotation.w = 0.9238795;


        tf_base_link_to_POI_frame.header.stamp = this->get_clock()->now();
        tf_base_link_to_POI_frame.header.frame_id = "base_link";
        tf_base_link_to_POI_frame.child_frame_id = "POI_frame";

        tf_base_link_to_POI_frame.transform.translation.x = 0.0;
        tf_base_link_to_POI_frame.transform.translation.y = 1.0;
        tf_base_link_to_POI_frame.transform.translation.z = 0.0;

        tf_base_link_to_POI_frame.transform.rotation.x = 0.0;
        tf_base_link_to_POI_frame.transform.rotation.y = 0.0;
        tf_base_link_to_POI_frame.transform.rotation.z = 0.0;
        tf_base_link_to_POI_frame.transform.rotation.w = 1.0;


/*
step 2.
'StaticTransformBroadcaster' 또는 'TransformBroadcaster' 객체를 이용하여 위에서 정의한 'TransformStamped' msg를 발행한다.
이때, 두 frame간 tf가 고정적이라면 'StaticTransformBroadcaster' 객체를 통해 발행하고,
두 frame간 tf가 동적이라면 'TransformBroadcaster' 객체를 통해 발행하면 된다.
*/
        tf_static_broadcaster_->sendTransform(tf_map_to_base_link);
        tf_broadcaster_->sendTransform(tf_base_link_to_POI_frame);


        pub_tf_signal_->publish(tf_signal_msg);
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<TFPubNode>();

    executor.add_node(node);
    executor.spin();


    rclcpp::shutdown();

    return 0;
}