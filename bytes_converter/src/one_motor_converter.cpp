#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

template <typename T>
std::vector<uint8_t> serialize(const T& data) {
    std::vector<uint8_t> bytes(sizeof(data));
    std::memcpy(bytes.data(), &data, sizeof(data));
    return bytes;
}

class BytesConverter : public rclcpp::Node
{
  public:
    BytesConverter()
    : Node("bytes_converter")
    {
      publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/output", 10);
      subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "/input", 10, std::bind(&BytesConverter::callback, this, _1));
    }

  private:
    void callback(const std_msgs::msg::Float32 sub_msg)
    {
      auto get_data = sub_msg.data;

      RCLCPP_INFO(this->get_logger(), "Send:%.5lf", sub_msg.data);

      auto msg = std_msgs::msg::UInt8MultiArray();
      msg.data = serialize(get_data);
      msg.data.insert(msg.data.begin(),'s');
      msg.data.push_back('e');
      msg.data.push_back('\n');
      publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BytesConverter>());
  rclcpp::shutdown();
  return 0;
}