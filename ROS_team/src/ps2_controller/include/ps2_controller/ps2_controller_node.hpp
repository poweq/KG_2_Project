// ps2_controller_node.hpp

#ifndef PS2_CONTROLLER_NODE_HPP
#define PS2_CONTROLLER_NODE_HPP

#include "ps2_controller/ps2_controller.h"
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PS2ControllerNode : public rclcpp::Node
{
public:
    PS2ControllerNode();
    ~PS2ControllerNode();

private:
    void read_loop();

    // 멤버 변수 선언
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    int ps2_fd_;
    ps2_map_t map_;
    std::thread reading_thread_;
};

#endif // PS2_CONTROLLER_NODE_HPP
