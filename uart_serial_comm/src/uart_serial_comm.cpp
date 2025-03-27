#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <vector>
#include <chrono>
#include <iostream>
#include <iomanip>

// 计算校验和函数
uint8_t calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum += data[i];
    }
    return checksum;
}

class UartSerialComm : public rclcpp::Node {
public:
    UartSerialComm() : Node("uart_serial_comm") {
        try {
            serial_port_.setPort("/dev/ttyUSB0");
            serial_port_.setBaudrate(9600);
            serial_port_.setBytesize(serial::eightbits);
            serial_port_.setStopbits(serial::stopbits_one);
            serial_port_.setParity(serial::parity_none);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(to);
            serial_port_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port: %s", e.what());
        }

        if (serial_port_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        }

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,[this](const geometry_msgs::msg::Twist::SharedPtr msg) 
        {
                if (serial_port_.isOpen()) {
                    std::vector<uint8_t> packet;
                    // 数据段，将 linear_x 和 angular_z 放大 10 倍并使用有符号整数类型
                    int16_t linear_x = static_cast<int16_t>(msg->linear.x * 10);
                    int16_t angular_z = static_cast<int16_t>(msg->angular.z * 10);
                    // 将有符号整数转换为大端字节序添加到数据包中
                    packet.push_back(static_cast<uint8_t>(linear_x >> 8));
                    packet.push_back(static_cast<uint8_t>(linear_x & 0xFF));
                    packet.push_back(static_cast<uint8_t>(angular_z >> 8));
                    packet.push_back(static_cast<uint8_t>(angular_z & 0xFF));

                    packet.push_back(calculateChecksum(packet.data(), packet.size()));

                    std::cout << "Sending data frame: ";
                    for (uint8_t byte : packet) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                        try {
                            std::vector<uint8_t> single_byte(1, byte);
                            serial_port_.write(single_byte);
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        } catch (serial::IOException& e) {
                            RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", e.what());
                        }
                    }
                    std::cout << std::endl;
                }
            });
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    serial::Serial serial_port_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UartSerialComm>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}    