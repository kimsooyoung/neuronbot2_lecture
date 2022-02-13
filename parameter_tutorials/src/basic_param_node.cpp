// reference from robotics backend
// https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/

#include "rclcpp/rclcpp.hpp"

class TestParams : public rclcpp::Node
{
public:
  TestParams() : Node("test_params_rclcpp") {
    
    this->declare_parameter("my_str", "Default for string");
    this->declare_parameter("my_int", 42);
    this->declare_parameter("my_double_array", std::vector<double>{7.7, 8.8});

    rclcpp::Parameter str_param = this->get_parameter("my_str");
    rclcpp::Parameter int_param = this->get_parameter("my_int");
    rclcpp::Parameter double_array_param = this->get_parameter("my_double_array");
    
    // rclcpp::Parameter str_param;
    // rclcpp::Parameter int_param;
    // rclcpp::Parameter double_array_param; 

    // exception handle
    // this->get_parameter("my_str", str_param);
    // this->get_parameter("my_int", int_param);
    // this->get_parameter("my_double_array", double_array_param);

    std::string my_str = str_param.as_string();
    int my_int = int_param.as_int();
    std::vector<double> my_double_array = double_array_param.as_double_array();
    
    RCLCPP_INFO(this->get_logger(), "str: %s, int: %s, double[]: %s",
                str_param.value_to_string().c_str(),
                int_param.value_to_string().c_str(),
                double_array_param.value_to_string().c_str());
    
  }
private:
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestParams>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}