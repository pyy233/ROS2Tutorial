#include "rclcpp/rclcpp.hpp"
#include "my_interface/msg/robot.hpp"

class TopicPublisher01 : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
        // 创建发布者，指定该发布者发布的数据类型和发布的话题名称command
        command_publisher_ = this->create_publisher<my_interface::msg::Robot>("robot", 10);
        // 创建定时器，500ms为周期，定时调用timer_callback回调函数
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),    std::bind(&TopicPublisher01::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 创建消息
        my_interface::msg::Robot message;
        message.name = "A";
        message.attribute.push_back(1);
        message.attribute.push_back(2);
        message.twist.angular.x = 1;
        message.twist.angular.y = 2;
        message.twist.angular.z = 3;
        message.twist.linear.x = 4;
        message.twist.linear.y = 5;
        message.twist.linear.z = 6;
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.name.c_str());
        command_publisher_->publish(message);
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者指针
    rclcpp::Publisher<my_interface::msg::Robot>::SharedPtr command_publisher_;
};

int main(int argc, char **argv)
{
    //初始化ros系统
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicPublisher01>("topic_publisher_01");
    //spin()函数，会让程序阻塞在这一步并开始不停地检测被spin的节点中有没有（与它的计时器或者订阅者绑定的）回调函数被触发
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}