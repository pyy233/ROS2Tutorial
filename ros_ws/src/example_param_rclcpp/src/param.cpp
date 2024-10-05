#include  <rclcpp/rclcpp.hpp>

class ParamNode: public rclcpp::Node
{
    public:
        ParamNode(std::string name):Node(name){
            this->declare_parameter("my_param", -1);
            this->get_parameter("my_param", my_param);
            RCLCPP_INFO(this->get_logger(), "Parameter my_param: %d", my_param);
            // RCLCPP_INFO(rclcpp::get_logger("11"), "Parameter my_param: %d", my_param);
        }
    private:
        int my_param;
};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<ParamNode>("param_node");
    rclcpp::shutdown();
    return 0;
}