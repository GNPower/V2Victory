#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "car_pkg/msg/carmsg.hpp"
#include "car_pkg/msg/intersectionmsg.hpp"

using namespace std::chrono_literals;

struct {
  uint32_t position_x;
  uint32_t position_y;
  uint32_t heading;
  float vehicle_speed;
  uint8_t priority;
} CarMsg;

struct {
  uint32_t position_x;
  uint32_t position_y;
  uint8_t num_directions;
  uint32_t directions[32];
  uint8_t intersection_state;
  uint8_t intersection_next_state;
  float intersection_switch_time;
} IntersectionMsg;


class IntersectionMessager : public rclcpp::Node
{
  public:
    IntersectionMessager()
    : Node("intersection_messager")
    {
      publisher_ = this->create_publisher<car_pkg::msg::intersectionmsg>("intersection_data", 10);
      subscription_ = this->create_subscription<std_msgs::msg::intersectionmsg>("car_data", 10, std::bind(&IntersectionMessager::car_callback, this, _1));
    }

    void publish(struct IntersectionMsg IntersectionMsg)
    {
      auto message = car_pkg::msg::intersectionmsg();
      message.position_x = IntersectionMsg.position_x;
      message.position_y = IntersectionMsg.position_y;
      message.num_directions = IntersectionMsg.num_directions;
      message.directions = IntersectionMsg.directions;
      message.intersection_state = IntersectionMsg.intersection_state;
      message.intersection_next_state = IntersectionMsg.intersection_next_state;
      message.intersection_switch_time = IntersectionMsg.intersection_switch_time;

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%d'", message.vehicle_speed, message.heading);
      publisher_->publish(message);
    }

    private:
      void car_callback(const car_pkg::msg::intersectionmsg & msg) const
      {
        struct CarMsg CMsg;
        CMsg.position_x = msg->position_x;
        CMsg.position_y = msg->position_y;
        CMsg.heading = msg->heading;
        CMsg.vehicle_speed = msg->vehicle_speed;
        CMsg.priority = msg->priority;

        // RCLCPP_INFO(this->get_logger(), "Intersection State: %d", msg->intersection_state);
        // RCLCPP_INFO(this->get_logger(), "Intersection Next State: %d", msg->intersection_next_state);
        // RCLCPP_INFO(this->get_logger(), "Intersection Switch Time: %f", msg->intersection_switch_time);\

        //TODO: CALL CALLBACK AND PASS IN STRUCTURE
      }

      rclcpp::Publisher<car_pkg::msg::intersectionmsg>::SharedPtr publisher_;
      rclcpp::Subscription<car_pkg::msg::carmsg>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<IntersectionMessager>());

  rclcpp::shutdown();
  return 0;
}