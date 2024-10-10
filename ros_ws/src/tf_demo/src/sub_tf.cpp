#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TFListenerNode : public rclcpp::Node
{
public:
  TFListenerNode() : Node("tf_listener_node")
  {
    // 创建 Buffer 和 TransformListener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 定时检查并打印坐标系变换
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TFListenerNode::lookup_transform, this)
    );
  }

private:
  void lookup_transform()
  {
    try
    {
      // 查找 "world" 到 "robot" 的坐标系变换
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform("world", "robot", tf2::TimePointZero);

      // 打印变换
      RCLCPP_INFO(this->get_logger(), "Got transform: translation (%.2f, %.2f, %.2f)",
                  transform_stamped.transform.translation.x,
                  transform_stamped.transform.translation.y,
                  transform_stamped.transform.translation.z);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFListenerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}