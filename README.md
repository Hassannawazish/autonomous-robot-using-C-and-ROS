<a href="https://sonarcloud.io/summary/overall?id=Hassannawazish_turtlebot3_localization_ros2&branch=main" 
   title="Click to view SonarCloud project overview">
    <img src="https://sonarcloud.io/images/project_badges/sonarcloud-black.svg" alt="SonarCloud badge">
</a>

# Autonomous robot using C++ and ROS
The goal is to complete the basics of ROS2 using C++. Without using or minimal use of GPT.
The package structure is given below,

![Image description](data/ros-1.png)

The package is created by using the command below,

$ros2 pkg create --build-type ament_cmake <package_name> --dependencies <package_dependency_1> <package_dependency_2>

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/
cd src
ros2 pkg create --build-type ament_cmake package_name --dependencies rclcpp
```

For building the package,

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/
colcon build
```
or you can use like that,

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/
colcon build --packages-select <package_name>
```


