#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

struct send_data{
    std::uint8_t id;
    float motor_0;
    float motor_1;
    float motor_2;
    float motor_3;
};

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
      subscription_0 = this->create_subscription<std_msgs::msg::Float32>(
      "/motor_0", 10, std::bind(&BytesConverter::callback_0, this, _1));

      subscription_1 = this->create_subscription<std_msgs::msg::Float32>(
      "/motor_1", 10, std::bind(&BytesConverter::callback_1, this, _1));

      subscription_2 = this->create_subscription<std_msgs::msg::Float32>(
      "/motor_2", 10, std::bind(&BytesConverter::callback_2, this, _1));

      subscription_3 = this->create_subscription<std_msgs::msg::Float32>(
      "/motor_3", 10, std::bind(&BytesConverter::callback_3, this, _1));

      timer_ = this->create_wall_timer(20ms , [this](){
        this->timer_callback();
      });

      data = send_data();
    }

  private:
    void callback_0(const std_msgs::msg::Float32 sub_msg)
    {

      data.motor_0 = sub_msg.data;
    }
    void callback_1(const std_msgs::msg::Float32 sub_msg)
    {
      data.motor_1 = sub_msg.data;
    }
    void callback_2(const std_msgs::msg::Float32 sub_msg)
    {
      data.motor_2 = sub_msg.data;
    }
    void callback_3(const std_msgs::msg::Float32 sub_msg)
    {
      data.motor_3 = sub_msg.data;
    }

    void timer_callback(){
      RCLCPP_INFO(this->get_logger(), "fl:%.5lf fr:%.5lf rl:%.5lf rr:%.5lf \n", data.motor_0, data.motor_1, data.motor_2, data.motor_3);

      auto msg = std_msgs::msg::UInt8MultiArray();
      msg.data = serialize(data);
      msg.data.insert(msg.data.begin(),'t');
      msg.data.insert(msg.data.begin(),'s');
      msg.data.push_back('e');
      msg.data.push_back('n');
      publisher_->publish(msg);

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_0;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_1;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_2;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_3;

    send_data data;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BytesConverter>());
  rclcpp::shutdown();
  return 0;
}