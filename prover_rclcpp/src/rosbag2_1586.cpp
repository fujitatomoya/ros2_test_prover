#include <memory>
#include <chrono>
#include <iostream>

#include <rosbag2_cpp/writer.hpp>

class RosbagSaver
{
private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_ros_ { nullptr };
public:
  bool Init();
};

bool RosbagSaver::Init()
{
  writer_ros_ = std::make_unique<rosbag2_cpp::Writer>();
  std::string rosbag_dir = "/root/ros2_ws/colcon_ws/testbag";
  if (writer_ros_) {
      writer_ros_->open(rosbag_dir);
  }
  std::cout << "!!!!!!!!!!" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  
  rosbag2_cpp::writer_interfaces::BaseWriterInterface& impl = writer_ros_->get_implementation_handle();
  std::cout << "111111111111" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  impl.close();  // sometimes raise "Segmentation Error"
  std::cout << "33333333333333333" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  writer_ros_.reset();
  std::cout << "************" << std::endl;

  return true;
}

int main()
{
  std::shared_ptr<RosbagSaver> a_ptr;
  a_ptr = std::make_shared<RosbagSaver>();
  if (!a_ptr->Init()) std::cout << "@@@@@@@@@@@@@@@@@@@@" << std::endl;

  std::cout << "!!!!!!" << std::endl;

  return 0;
}