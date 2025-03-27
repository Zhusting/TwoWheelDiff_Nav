#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>

class MapToOdomBroadcaster : public rclcpp::Node
{
public:
    MapToOdomBroadcaster()
        : Node("map_to_odom_broadcaster")
    {
        // 创建 TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 定时器，用于定期发布 TF
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz 发布频率
            std::bind(&MapToOdomBroadcaster::publish_transform, this));
    }

private:
    void publish_transform()
    {
        // 创建一个 TransformStamped 消息
        geometry_msgs::msg::TransformStamped transform_stamped;

        // 设置时间戳和坐标系
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "odom";

        // 设置平移部分 (x, y, z)
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;

        // 设置旋转部分 (四元数)
        transform_stamped.transform.rotation.x = 0.0;
        transform_stamped.transform.rotation.y = 0.0;
        transform_stamped.transform.rotation.z = 0.0;
        transform_stamped.transform.rotation.w = 1.0;

        // 发布 TF
        tf_broadcaster_->sendTransform(transform_stamped);
    }

    // 成员变量
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    // 初始化 ROS 2 节点
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapToOdomBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
