source ../../.bashrc
source car_interface/install/setup.bash
colcon build
source install/setup.bash
ros2 run vehicle_main tm
