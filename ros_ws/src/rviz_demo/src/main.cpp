#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class SquarePublisher : public rclcpp::Node
{
public:
  SquarePublisher() : Node("square_publisher")
  {
    // 创建发布者，发布 "visualization_marker" 话题
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // 定时发布正方形 Marker
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SquarePublisher::publish_square_marker, this)
    );
  }

private:
  void publish_square_marker()
  {
    auto marker = visualization_msgs::msg::Marker();
    
    // Marker的基础信息
    marker.header.frame_id = "map"; // 使用"map"作为坐标系
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "square";
    marker.id = 0;  // 唯一ID
    marker.type = visualization_msgs::msg::Marker::CUBE;  // 选择CUBE类型来表示正方形
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 设置正方形的位置
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // 设置正方形的大小
    marker.scale.x = 1.0;  // 设置边长为1.0
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // 设置颜色（RGBA）
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;  // 透明度

    // 发布Marker
    publisher_->publish(marker);

    RCLCPP_INFO(this->get_logger(), "Publishing square marker");
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquarePublisher>());
  rclcpp::shutdown();
  return 0;
}