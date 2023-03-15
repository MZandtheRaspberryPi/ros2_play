# Get Docker Image
```
docker pull osrf/ros:foxy-desktop

xhost +local:docker

docker run -it --env DISPLAY=${DISPLAY} --volume /tmp/.X11-unix:/tmp/.X11-unix --network host osrf/ros:foxy-desktop 
```

## ROS 2 new topics
* domain id, can have multipe groups on one network, with diff domain ids. 0-101, env var: 
* export ROS_DOMAIN_ID=<your_domain_id>
* if want only localhost: export ROS_LOCALHOST_ONLY=1
* remapping: `ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel`, now via --ros-args --.
* ros2, msg became interface. ros2 interface show geometry_msgs/msg/Twist
* parameters, can dump easily: `ros2 param dump /turtlesim`, loading on startup too `--ros-args --params-file <file_name>`
* actions, long running, pre-emptable (can cancel), give regular updates. server or client can abbort. `ros2 action list`, `ros2 action send_goal`
* view logs with rqt_console.
* launch files can be python, xml, yaml.
```
# turtlesim/launch/multisim.launch.py

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen'),
    ])

```

* ros2 bag is still a thing
* colcon iteration on build tools. `sudo apt install python3-colcon-common-extensions`
  * build is where intermediate files stored. subfolder for each package, where cmake invoked.
  * install, where package installed to
  * log dir, varioius logging info about each colcon
  * no devel, compared to catkin
* overlay vs underlay. if making new package, may make as new overlay onto ros2 general packages underlay.
* ament_cmake may not support devel, requires install, colcon has option: `colcon build --symlink-install`
* colcon supports ament_cmake, and ament_python, and pure cmake.
* source install/setup.bash
* making packag,e can pass node name to make hello world node. also pass build type. 
  * `ros2 pkg create --build-type ament_cmake --node-name my_node my_package`
* CMake familiar, but ament_cmake
```
root@mz-VirtualBox:~/ros2_ws/src/my_package# cat CMakeLists.txt 
cmake_minimum_required(VERSION 3.5)
project(my_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(my_node src/my_node.cpp)
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```
* cpp publisher. We use std::bind to bind a callback.

```
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```
* ros2 uses c++ 11 and sometimeas c++ 14
* logging a bit diff: `
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());`
* std::make_shared<MyNode>() used now
* spin takes a shared pointer as an arg for what cbs and all
* subscriber, note we use placeholder: `subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));`, `using std::placeholders::_1;`
* `void topic_callback(const std_msgs::msg::String::SharedPtr msg) const`
* then we use add_executable, then ament_target_dependencies().
* using parameters in a class, declare parameter, then getparameter.get_parameter_value, then get<std::string>. and also can set.
```
 #include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    this->declare_parameter("my_parameter", "world");

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback()
  {
    std::string my_param =
      this->get_parameter("my_parameter").get_parameter_value().get<std::string>();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}

```
* can add descriptor to give it name, range, read only, ect.
```
auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is mine!";
this->declare_parameter("my_parameter", "world", param_desc);
```
* generating new action use rosidl to generate code: 
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```
* An action server requires 6 things:

    The templated action type name: Fibonacci.

    A ROS 2 node to add the action to: this.

    The action name: 'fibonacci'.

    A callback function for handling goals: handle_goal

    A callback function for handling cancellation: handle_cancel.

    A callback function for handling goal accept: handle_accept.





## commands
* ros2 node list, ros2 topic list, ros2 service list, ros2 action list.
* remapping: ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
* ROS2, single executable can contain multiple nodes.
* ros2 node info /my_turtle