#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class TFBroadcasterNode : public rclcpp::Node
{
public:
  TFBroadcasterNode() : Node("tf_broadcaster_node")
  {
    // 创建 TransformBroadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 定时发布 TF 变换，每 100 毫秒发布一次
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TFBroadcasterNode::broadcast_transform, this)
    );
  }

private:
  void broadcast_transform()
  {
    // 创建 TransformStamped 消息
    geometry_msgs::msg::TransformStamped transform_stamped;

    // 设置坐标系 ID 和时间戳
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "world";  // 父坐标系
    transform_stamped.child_frame_id = "robot";   // 子坐标系

    // 设置位移 (translation)
    transform_stamped.transform.translation.x = 1.0;
    transform_stamped.transform.translation.y = 2.0;
    transform_stamped.transform.translation.z = 0.0;

    // 设置旋转 (四元数表示)
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;

    // 发布 TF 变换
    tf_broadcaster_->sendTransform(transform_stamped);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFBroadcasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}