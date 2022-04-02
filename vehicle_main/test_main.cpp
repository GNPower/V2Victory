#include "stdio.h"
#include "time.h"

#include "test_top.h"
#include "localize/localize.h"
#include "motor_drive/motor_drive.h"
#include "encoder/encode.h"

#include <pthread.h>
#include <signal.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "car_interface/msg/car.hpp"
#include "car_interface/msg/intersection.hpp"

using namespace std::chrono_literals;

#define DUTY 60
#define HEADING 0


////////////////////////////COM CLASSES///////////////////////////////////////////////////
class CarMessager : public rclcpp::Node
{
  public:
    CarMessager()
    : Node("car_messager")
    {
      publisher_ = this->create_publisher<car_interface::msg::Car>("car_data", 10);
      subscription_ = this->create_subscription<car_interface::msg::Intersection>("intersection_data", 10, std::bind(&CarMessager::intersection_callback, this, std::placeholders::_1));
    }

    void publish(struct Vehicle_Data Vehicle_Data)
    {
      auto message = car_interface::msg::Car();
      message.position_x = Vehicle_Data.position_x;
      message.position_y = Vehicle_Data.position_y;
      message.heading = Vehicle_Data.heading;
      message.vehicle_speed = Vehicle_Data.speed;
      message.priority = Vehicle_Data.priority;
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%d'", message.vehicle_speed, message.heading);
      publisher_->publish(message);
    }

  private:
    void intersection_callback(const car_interface::msg::Intersection::SharedPtr msg)
    {
      struct Intersection_Data IMsg;
      IMsg.position_x = msg->position_x;
      IMsg.position_y = msg->position_y;
      IMsg.num_directions = msg->num_directions;
      uint32_t directions = msg->directions;
      IMsg.directions[0] = (directions >>  0) & 0xFF;
      IMsg.directions[1] = (directions >>  8) & 0xFF;
      IMsg.directions[2] = (directions >> 16) & 0xFF;
      IMsg.directions[3] = (directions >> 24) & 0xFF;
      IMsg.intersection_state = msg->intersection_state;
      IMsg.intersection_next_state = msg->intersection_next_state;
      IMsg.intersection_switch_time = msg->intersection_switch_time;

      // RCLCPP_INFO(this->get_logger(), "Intersection State: %d", msg->intersection_state);
      // RCLCPP_INFO(this->get_logger(), "Intersection Next State: %d", msg->intersection_next_state);
      // RCLCPP_INFO(this->get_logger(), "Intersection Switch Time: %f", msg->intersection_switch_time);

      //TODO: CALL CALLBACK AND PASS IN STRUCTURE
    }

    rclcpp::Publisher<car_interface::msg::Car>::SharedPtr publisher_;
    rclcpp::Subscription<car_interface::msg::Intersection>::SharedPtr subscription_;
};
/////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char *argv[]){
	pthread_t left_tid, right_tid;
	float distance_x, distance_y;
	int duty_a = DUTY;
	int duty_b = DUTY;



	//INITS///////////////////////////////////////////////////////////////////////////////////
	
	if (2 == GPIO_init(duty_a, duty_b)){
		printf("Error Initiating GPIOs");
	}
	
	init_encoders(&left_tid, &right_tid);
	
        rclcpp::init(argc, argv);
	////////////////////////////////////////////////////////////////////////////////////////////////

	Vehicle_Data ego;
	Intersection_Data intersection;

	ego.position_x = 0;
	ego.position_y = 0;
	ego.heading = HEADING;

	intersection.position_x = 100;
	intersection.position_y = 200;

        CarMessager Test;
	///////////////////////////////////////////////////////////////////////////////////////////

	/*
	printf("x: %d  y: %d\n", ego.position_x, ego.position_y);
	update_location(&ego, 2, 7);
	printf("x: %d  y: %d\n", ego.position_x, ego.position_y);

	float distance;
	get_abs_distance(&ego, &intersection, &distance);
	printf("dist: %f\n", distance);
	*/
	

	printf("No Sleep Till Brooklyn \n");

	int count = 0;
	clock_t past_time = clock();
	clock_t current_time = clock();
	float time_passed= 0;
	
	set_forward();

	while(1){
		Test.publish(ego);
		rclcpp::spin_some(std::make_shared<CarMessager>());
		
		
		if(GPIORead(STOP)) break;

		current_time = clock();
		time_passed = (float)(current_time - past_time)/CLOCKS_PER_SEC;
		past_time = current_time;


		get_x_distance_traveled(&ego, &distance_x);
		//printf("X_t: %f\t", distance_x);
		get_y_distance_traveled(&ego, &distance_y);
		//printf("Y_t: %f\t", distance_y);
		update_location(&ego, distance_x, distance_y, time_passed);
		

		if (count > 100){
			count = 0;
			printf("X: %d, Y: %d Speed (mm/s): %f TimeStep: %f\n", ego.position_x, ego.position_y, ego.speed, time_passed);
		}

		count++;
		usleep(TIMESTEP);
	}

	if (2 == GPIO_init(duty_a, duty_b)){
		printf("Error Closing GPIOs");
	}

	rclcpp::shutdown();
	pthread_kill(left_tid, SIGKILL);
	pthread_kill(right_tid, SIGKILL);


	return 0;
}
