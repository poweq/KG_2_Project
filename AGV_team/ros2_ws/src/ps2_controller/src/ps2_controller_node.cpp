// ps2_controller/src/ps2_controller_node.cpp

#include "ps2_controller/ps2_controller_node.hpp"
#include "ps2_controller/ps2_controller.h" // 기존 ps2_controller.h도 필요
#include <cmath> // fabs 함수 사용

double normalize_axis(int value)
{
    return static_cast<double>(value) / 32767.0;
}

// 생성자 구현
PS2ControllerNode::PS2ControllerNode()
: Node("ps2_controller_node")
{
    RCLCPP_INFO(this->get_logger(), "PS2ControllerNode has been started.");

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    ps2_fd_ = ps2_open("/dev/input/js0");
    if(ps2_fd_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/input/js0");
        return;
    }

    memset(&map_, 0, sizeof(ps2_map_t));
    reading_thread_ = std::thread(&PS2ControllerNode::read_loop, this);
}

// 소멸자 구현
PS2ControllerNode::~PS2ControllerNode()
{
    if(reading_thread_.joinable())
    {
        reading_thread_.join();
    }
    ps2_close(ps2_fd_);
}

// read_loop 함수 구현
void PS2ControllerNode::read_loop()
{
    RCLCPP_INFO(this->get_logger(), "Starting read loop");
    int len;
    geometry_msgs::msg::Twist msg;

    // 최대 속도 설정
    const double max_linear_speed = 1.0;   // 최대 선속도 (m/s)
    const double max_angular_speed = 1.0;  // 최대 각속도 (rad/s)

    while(rclcpp::ok())
    {
        len = ps2_map_read(ps2_fd_, &map_);
        if (len < 0)
        {
            usleep(10*1000);
            continue;
        }

        // 아날로그 스틱 값 읽기
        int lx = map_.lx;
        int ly = map_.ly;

        // 로그 출력 (디버깅 용도)
        RCLCPP_INFO(this->get_logger(), "Raw axis values: lx=%d, ly=%d", lx, ly);

        // 축 값 정규화
        double normalized_lx = normalize_axis(lx);
        double normalized_ly = normalize_axis(ly);

        // 데드존 적용
        double deadzone = 0.1;

        if (fabs(normalized_lx) < deadzone)
            normalized_lx = 0.0;
        if (fabs(normalized_ly) < deadzone)
            normalized_ly = 0.0;

        // 축 값 반전 (필요한 경우)
        normalized_ly = -normalized_ly; // 위쪽으로 움직이면 양수

        // 속도 계산
        double linear_x = normalized_ly * max_linear_speed;
        double angular_z = normalized_lx * max_angular_speed;

        // 매우 작은 값은 0으로 처리
        if (fabs(linear_x) < 1e-3)
            linear_x = 0.0;
        if (fabs(angular_z) < 1e-3)
            angular_z = 0.0;

        // 메시지 설정
        msg.linear.x = linear_x;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = angular_z;

        // 메시지 퍼블리시
        pub_->publish(msg);

        // 로그 출력
        RCLCPP_INFO(this->get_logger(), "Published Twist message: linear.x=%.2f, angular.z=%.2f", msg.linear.x, msg.angular.z);

        // 주기 유지
        usleep(20 * 1000); // 50Hz
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PS2ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
