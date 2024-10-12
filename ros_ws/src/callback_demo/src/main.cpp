#include "rclcpp/rclcpp.hpp"
#include <chrono>

class CallbackNode : public rclcpp::Node
{
public:
    CallbackNode(std::string name) : Node(name)
    {
        reentrant_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        mutually_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_1 = this->create_wall_timer(std::chrono::microseconds(1),
                                            std::bind(&CallbackNode::timer1_callback, this),
                                            reentrant_callback_group);
        timer_2 = this->create_wall_timer(std::chrono::microseconds(1),
                                            std::bind(&CallbackNode::timer2_callback, this),
                                            reentrant_callback_group);
        timer_3 = this->create_wall_timer(std::chrono::microseconds(1),
                                            std::bind(&CallbackNode::timer3_callback, this),
                                            reentrant_callback_group);

    }

private:
    rclcpp::CallbackGroup::SharedPtr reentrant_callback_group;
    rclcpp::CallbackGroup::SharedPtr mutually_callback_group;
    
    void timer1_callback()
    {
        std::this_thread::sleep_for(std::chrono::nanoseconds(1));
        if(timer1_times >= 100000){
            return;
        }
        timer1_times++;
        a++;
    }
    void timer2_callback()
    {
        if(timer2_times >= 100000){
            return;
        }
        timer2_times++;
        a++;
    }
    void timer3_callback()
    {
        if(timer3_times >= 200000){
            return;
        }
        timer3_times++;
        b++;
    }
    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::TimerBase::SharedPtr timer_2;
    rclcpp::TimerBase::SharedPtr timer_3;
    public:
        int a = 0;
        int b = 0;
        int timer1_times = 0;
        int timer2_times = 0;
        int timer3_times = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CallbackNode>("callbacknode");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    std::cout<<"a = " << node-> a << std::endl;
    std::cout<<"b = " << node-> b << std::endl;
    rclcpp::shutdown();
    return 0;
}