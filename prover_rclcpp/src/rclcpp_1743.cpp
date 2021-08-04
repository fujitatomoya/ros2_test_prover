#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;

class ParametersClass: public rclcpp::Node
{
  public:
    ParametersClass()
      : Node("parameter_node")
    {
#if 0 // these work okay
      this->declare_parameter<int8_t>("this_int_works1", 1);
      this->declare_parameter<int16_t>("this_int_works2", 1);
      this->declare_parameter<int32_t>("this_int_works3", 1);
      this->declare_parameter<int64_t>("this_int_works4", 1);

      this->declare_parameter<uint8_t>("this_uint_works1", 1);
      this->declare_parameter<uint16_t>("this_uint_works2", 1);
#endif
      this->declare_parameter<uint32_t>("this_uint_does_not_work1", 1);
      this->declare_parameter<uint64_t>("this_uint_does_not_work2", 1);
    }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametersClass>());
  rclcpp::shutdown();
  return 0;
}
