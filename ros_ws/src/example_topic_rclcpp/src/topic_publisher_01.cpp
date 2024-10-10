#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicPublisher01 : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
        // 创建发布者，指定该发布者发布的数据类型和发布的话题名称command
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability();
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
        // command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", qos);
        // 创建定时器，500ms为周期，定时调用timer_callback回调函数
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),    std::bind(&TopicPublisher01::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 创建消息
        std_msgs::msg::String message;
        message.data = "forward";
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // 发布消息
        command_publisher_->publish(message);
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者指针
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
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