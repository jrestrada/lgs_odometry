#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#define LOG(...) RCLCPP_INFO(rclcpp::get_logger("behavior tree"), __VA_ARGS__)

class ImuPositionEstimatorNode : public rclcpp::Node
{
public:
  ImuPositionEstimatorNode()
    : Node("imu_position_estimator_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&ImuPositionEstimatorNode::imu_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_integration", 10);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    // auto curr_timestamp_ = rclcpp::Time(imu_msg->header.stamp);
    // uint64_t delta_t = curr_timestamp_.nanoseconds() - last_timestamp_.nanoseconds();

    // LOG("curr time %i", curr_timestamp_.nanoseconds());
    // LOG("last time %i", last_timestamp_.nanoseconds());
    // LOG("delta %i", delta_t);
    // double delta_t_s = delta_t / 1000000000;
    double delta_t_s = 0.2;
    double acceleration_x = imu_msg->linear_acceleration.x;
    double velocity_x = (acceleration_x == 0.0) ? 0.0 : last_velocity_x_ + acceleration_x * delta_t_s;
    last_velocity_x_ = velocity_x;

    double position_x = last_position_x_ + velocity_x * delta_t_s;
    last_position_x_ = position_x;

    std_msgs::msg::Float32 position_msg;
    position_msg.data = position_x;
    publisher_->publish(position_msg);
    last_timestamp_ = imu_msg->header.stamp;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

  rclcpp::Time last_timestamp_;
  double last_velocity_x_ = 0.0;
  double last_position_x_ = 0.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuPositionEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}