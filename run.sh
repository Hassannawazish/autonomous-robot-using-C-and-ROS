cd ~/ros2_ws
source install/setup.bash
colcon build --packages-select autonomous_turtlebot
source install/setup.bash
ros2 run autonomous_turtlebot distance_finder