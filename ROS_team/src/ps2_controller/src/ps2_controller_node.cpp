// ps2_controller/src/ps2_controller_node.cpp

#include "ps2_controller/ps2_controller_node.hpp"
#include "ps2_controller/ps2_controller.h" // 기존 ps2_controller.h도 필요
#include <cmath> // fabs 함수 사용
#include <unistd.h> // usleep 함수 사용
#include <cstring> // memset 함수 사용

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

    // 로그 빈도 조절을 위한 변수
    int log_count = 0;

    while(rclcpp::ok())
    {
        len = ps2_map_read(ps2_fd_, &map_);
        if (len < 0)
        {
            usleep(10*1000); // 10ms 대기
            continue;
        }

        // 아날로그 스틱 값 읽기
        int rx = map_.rx; // 오른쪽 아날로그 x축
        int ly = map_.ly; // 왼쪽 아날로그 y축

        // L1, R1 버튼 값 읽기
        int l1 = map_.l1; // L1 버튼 상태 (0: Released, 1: Pressed)
        int r1 = map_.r1; // R1 버튼 상태 (0: Released, 1: Pressed)

        // 로그 출력 (디버깅 용도)
       // RCLCPP_INFO(this->get_logger(), "Raw axis values: rx=%d, ly=%d, L1=%d, R1=%d", 
        //    rx, ly, l1, r1);

        // 축 값 정규화
        double normalized_rx = normalize_axis(rx);
        double normalized_ly = normalize_axis(ly);

        // 데드존 적용
        double deadzone = 0.1;

        if (fabs(normalized_rx) < deadzone)
            normalized_rx = 0.0;
        if (fabs(normalized_ly) < deadzone)
            normalized_ly = 0.0;

        // 축 값 반전 (필요한 경우)
        normalized_ly = -normalized_ly; // 위쪽으로 움직이면 양수

        // 속도 계산
        double linear_x = normalized_ly * max_linear_speed;
        double angular_z = normalized_rx * max_angular_speed;
        double linear_y = 0.0;

        // L1과 R1 버튼을 이용한 수평 이동 제어
        if (l1 && !r1)
        {
            linear_y = -1.0; // 왼쪽으로 이동
        }
        else if (r1 && !l1)
        {
            linear_y = 1.0; // 오른쪽으로 이동
        }
        // 만약 두 버튼이 동시에 눌렸다면, 수평 이동을 하지 않도록 설정
        else
        {
            linear_y = 0.0;
        }

        // 매우 작은 값은 0으로 처리
        if (fabs(linear_x) < 1e-3)
            linear_x = 0.0;
        if (fabs(linear_y) < 1e-3)
            linear_y = 0.0;
        if (fabs(angular_z) < 1e-3)
            angular_z = 0.0;

        // 메시지 설정
        msg.linear.x = linear_x;
        msg.linear.y = linear_y;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = angular_z;

        // 메시지 퍼블리시
        pub_->publish(msg);

        // 로그 빈도 조절
        log_count++;
        if (log_count >= 10) // 5Hz로 로그 출력 (50Hz 루프에서 10회)
        {
            log_count = 0;
            RCLCPP_INFO(this->get_logger(), "Published Twist message: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f", 
                msg.linear.x, msg.linear.y, msg.angular.z);
        }

        // 주기 유지
        usleep(20 * 1000); // 20ms 대기, 50Hz 루프
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
