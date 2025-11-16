#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "spider_hardware_interface.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn SpiderHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    serial_port_ = info_.hardware_parameters["serial_port"];
    logger_ = rclcpp::get_logger("SpiderHardwareInterface");

    for (auto& joint : info_.joints) {
        motors_.emplace_back(std::stoi(joint.parameters.at("leg_id")), std::stoi(joint.parameters.at("motor_id")));
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn SpiderHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state) {
    if (!setup_serial_port(serial_port_, 0)) {
        RCLCPP_ERROR(logger_, "Can't open serial port %s", serial_port_.data());
        return CallbackReturn::ERROR;
    }
    node_->declare_parameter("motor1.p_gain", 0.0);
    return CallbackReturn::SUCCESS;
}

CallbackReturn SpiderHardwareInterface::on_cleanup(const rclcpp_lifecycle::State&) {
    if (serial_fd_) {
        ::close(serial_fd_);
    }
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SpiderHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &motors_[i].current_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &motors_[i].angle_
        ));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SpiderHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &motors_[i].angle_setpoint_
        ));
    }

    return command_interfaces;
}

bool SpiderHardwareInterface::setup_serial_port(const std::string& port, int baud_rate) {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Error opening %s: %s", port.c_str(), strerror(errno));
        return false;
    }
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(logger_, "Error getting termios attributes: %s", strerror(errno));
        return false;
    }
    // Set baud rate
    speed_t speed = B230400;
    if (cfsetospeed(&tty, speed) != 0) {
        RCLCPP_ERROR(logger_, "Error setting cfsetospeed");
        return false;
    }
    if (cfsetispeed(&tty, speed) !=0) {
        RCLCPP_ERROR(logger_, "Error setting cfsetispeed");
        return false;
    }

    // Configure port settings
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver

    // Local modes
    tty.c_lflag &= ~ICANON; // Non-canonical mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT, SUSP

    // Input modes
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    // Output modes
    tty.c_oflag &= ~OPOST; // Raw output
    tty.c_oflag &= ~ONLCR;

    // Read timeout settings
    tty.c_cc[VMIN] = 0;  // Minimum characters to read
    tty.c_cc[VTIME] = 1; // Timeout in deciseconds (0.1 seconds)

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(logger_, "Error setting termios attributes: %s", strerror(errno));
        return false;
    }
    tcflush(serial_fd_, TCIFLUSH);
    return true;
}

return_type SpiderHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    char buffer[256];
    ssize_t bytes_read;
    uint16_t pos = 0;
    char ch;        

    if (serial_fd_ >= 0) {    
        while ((bytes_read = ::read(serial_fd_, &ch, 1)) > 0) {
           if (ch == '\n' || ch == '\r' || pos >= sizeof(buffer) - 1) {
                buffer[pos] = '\0';

                // Process the complete line
                CanMessage cMsg;
                canMsgFromSlcan(buffer, cMsg);
                for (auto& motor : motors_) {
                    if (cMsg.address.legn == motor.leg_id_ && cMsg.address.motorn == motor.motor_id_) {
                        motor.processMsg(cMsg);
                    }
                }
                pos = 0;
            } else {
                buffer[pos++] = ch;
            }
        }
    } else {
        return return_type::ERROR;
    }
    
    return return_type::OK;
}

return_type SpiderHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)  {
    char buffer[256];
    size_t size;
    for (auto& motor : motors_) {
        for (auto& msg: motor.getMsgsToSend()) {
            size = canMsgToSlcan(msg, buffer, 256);
            ::write(serial_fd_, buffer, size);
        }
    };
    return return_type::OK;
}
