#include <chrono>
#include <iostream>
#include <functional>

#include "clobot_platform_libs/geometry/orientation.hpp"
#include "clobot_platform_libs/geometry/geometry.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // for tf2::doTransform()

#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using namespace std::chrono_literals;



/*
1 sec 주기로 'tf_signal' Topic을 받을 때마다 frame 간 tf 관계 체크
*/
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


/*
'TransformListener' 객체는 생성됨과 동시에 동일 ros 네트워크 상에 있는 모든 tf를 받는다.
그리고 받은 tf를 'Buffer' 객체에 최대 10초 동안 저장해둔다.
*/
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }


private:
    void on_subscribed_tf_signal(const std_msgs::msg::UInt8::SharedPtr tf_signal_msg)
    {
        geometry_msgs::msg::PoseStamped pose_in_map;
        geometry_msgs::msg::PoseStamped pose_in_base_link;
        geometry_msgs::msg::PoseStamped pose_in_POI_frame;


/*
step 1.
'PoseStamped' 타입의 msg를 생성한다.
하나는 source frame 에서의 좌표를 의미하는 msg로써 정의한다.
다른 하나는 구하고자 하는 target frame 에서의 좌표를 의미하는 msg로써 사용될 것이다.
*/
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


        pose_in_POI_frame.header.stamp = this->get_clock()->now();
        pose_in_POI_frame.header.frame_id = "POI_frame";

        pose_in_POI_frame.pose.position.x = 0;
        pose_in_POI_frame.pose.position.y = 0;
        pose_in_POI_frame.pose.position.z = 0;

        pose_in_POI_frame.pose.orientation.x = 0;
        pose_in_POI_frame.pose.orientation.y = 0;
        pose_in_POI_frame.pose.orientation.z = 0;
        pose_in_POI_frame.pose.orientation.w = 1;


/*
step 2.
source frame 에서의 좌표와 source frame <-> target frame 간의 tf 관계를 이용하여, target frame 에서의 좌표를 구한다.
*/
        try
        {
/*
'Buffer' 객체의 lookupTransform() 메소드를 활용하여 두 frame 간 tf 관계를 구한다.
아규먼트로는 'target frame 명', 'source frame 명', '언제 받아진 tf를 원하는지', 'tf를 탐색하기 위한 시간' 을 넣어준다.

3번째 인자로 tf2::TimePointZero 를 넣어주면 가장 최근에 받아진 tf 를 원하는 것이다. (tf2::TimePointZero 는 0초를 의미)

4번째 인자는 'Buffer' 객체가 tf를 탐색할 수 있도록 시간 여유를 주는 것이다.
'Buffer' 객체가 처음 생성되었을 때는 tf를 바로 받지 못할 수도 있다.
이때 4번째 인자로 시간 간격을 주면 lookupTranform() 메소드는 최대 해당 시간 간격동안 blocking이 되며 'Buffer' 객체가 tf를 받을 수 있는 시간적 여유를 준다.
'Buffer' 객체가 tf를 한번 받은 이후부터는 보통 blocking 되지 않는다.
4번째 인자는 'ms' 단위의 시간 간격을 받는다.
'tf2::Duration' 객체 형식으로 4번째 인자 값을 줄 수 있고, 'tf2::Duration' 객체는 'ns' 단위로 생성된다.
*/
            geometry_msgs::msg::TransformStamped tf_base_link_to_map = tf_buffer_->lookupTransform(
                "map",
                "base_link",
                tf2::TimePointZero,
                tf2::Duration(5000000)); // 5 ms


/*
tf2_geometry_msgs.h 헤더파일에 정의된 'tf2::doTransform()' 메소드를 활용하여 target frame 에서의 좌표를 구한다.
아규먼트로는 'source frame 에서의 좌표', 'target frame 에서의 좌표가 들어갈 msg', '두 frame 간의 tf' 를 넣어준다.
*/
            tf2::doTransform(pose_in_base_link, pose_in_map, tf_base_link_to_map);
        }


/*
'Buffer' 객체의 lookupTransform() 메소드가 정상 동작하지 않을 경우 (두 frame 간 tf 관계가 없을 경우) exception을 throw 한다.
*/
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

        euler.Yaw = TO_DEG(euler.Yaw);


        std::cout << "pose in base_link\t" << "x: " << pose_in_base_link.pose.position.x << " m, y: " << pose_in_base_link.pose.position.y
                    << " m, yaw: " << euler.Yaw << " deg" << std::endl;


        euler = clobot_platform_libs::geometry::Quaternion::ToEulerAngles(
            pose_in_map.pose.orientation.x,
            pose_in_map.pose.orientation.y,
            pose_in_map.pose.orientation.z,
            pose_in_map.pose.orientation.w
        );

        euler.Yaw = TO_DEG(euler.Yaw);


        std::cout << "=> pose in map\t" << "x: " << pose_in_map.pose.position.x << " m, y: " << pose_in_map.pose.position.y
                    << " m, yaw: " << euler.Yaw << " deg" <<std::endl;


        try
        {
            geometry_msgs::msg::TransformStamped tf_POI_frame_to_map = tf_buffer_->lookupTransform(
                "map",
                "POI_frame",
                tf2::TimePointZero,
                tf2::Duration(5000000)); // 5 ms

            tf2::doTransform(pose_in_POI_frame, pose_in_map, tf_POI_frame_to_map);
        }
        catch (const tf2::TransformException& ex)
        {
            std::cout << "POI_frame <-> map can't lookupTransform()" << std::endl;

            return;
        }


        euler = clobot_platform_libs::geometry::Quaternion::ToEulerAngles(
            pose_in_POI_frame.pose.orientation.x,
            pose_in_POI_frame.pose.orientation.y,
            pose_in_POI_frame.pose.orientation.z,
            pose_in_POI_frame.pose.orientation.w
        );

        euler.Yaw = TO_DEG(euler.Yaw);


        std::cout << "pose in POI_frame\t" << "x: " << pose_in_POI_frame.pose.position.x << " m, y: " << pose_in_POI_frame.pose.position.y
                    << " m, yaw: " << euler.Yaw << " deg" << std::endl;


        euler = clobot_platform_libs::geometry::Quaternion::ToEulerAngles(
            pose_in_map.pose.orientation.x,
            pose_in_map.pose.orientation.y,
            pose_in_map.pose.orientation.z,
            pose_in_map.pose.orientation.w
        );

        euler.Yaw = TO_DEG(euler.Yaw);


        std::cout << "=> pose in map\t" << "x: " << pose_in_map.pose.position.x << " m, y: " << pose_in_map.pose.position.y
                    << " m, yaw: " << euler.Yaw << " deg" << std::endl;

        std::cout << " ------------------------------------------ " << std::endl;
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<TFSubNode>();

    executor.add_node(node);
    executor.spin();


    rclcpp::shutdown();

    return 0;
}