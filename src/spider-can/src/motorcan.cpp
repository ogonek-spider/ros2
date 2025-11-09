#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <memory>

class SerialReaderNode : public rclcpp::Node
{
public:
    SerialReaderNode() : Node("serial_reader"), serial_fd_(-1)
    {
        // Declare parameters
        this->declare_parameter<std::string>("serial_port", "/dev/tty.usbmodem2101");
        this->declare_parameter<int>("baud_rate", 250000);

        // Get parameters
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        //Open and configure serial port
        if (!setupSerialPort(serial_port, baud_rate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
            rclcpp::shutdown();
            return;
        }

        // Create publisher
        serial_publisher_ = this->create_publisher<std_msgs::msg::String>("serial_data", 10);

        // Create reading thread
        read_thread_ = std::thread(&SerialReaderNode::readSerialThread, this);

        RCLCPP_INFO(this->get_logger(), "Serial reader node started on %s", serial_port.c_str());
    }

    ~SerialReaderNode()
    {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    bool setupSerialPort(const std::string& port, int baud_rate)
    {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error opening %s: %s", port.c_str(), strerror(errno));
            return false;
        }
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error getting termios attributes: %s", strerror(errno));
            return false;
        }
        // Set baud rate
        speed_t speed = B230400;
        if (cfsetospeed(&tty, speed) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error setting cfsetospeed");
            return false;
        }
        if (cfsetispeed(&tty, speed) !=0) {
            RCLCPP_ERROR(this->get_logger(), "Error setting cfsetispeed");
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
            RCLCPP_ERROR(this->get_logger(), "Error setting termios attributes: %s", strerror(errno));
            return false;
        }
        tcflush(serial_fd_, TCIFLUSH);
        return true;
    }
    
    void readSerialThread()
    {
        std::string buffer;
        char read_buffer[256];
        
        while (rclcpp::ok() && running_) {
            if (serial_fd_ >= 0) {
                ssize_t n = read(serial_fd_, read_buffer, sizeof(read_buffer) - 1);
                if (n > 0) {
                    read_buffer[n] = '\0';
                    buffer += std::string(read_buffer, n);
                    
                    // Split by \r and publish each complete message
                    size_t pos;
                    while ((pos = buffer.find('\r')) != std::string::npos) {
                        std::string message = buffer.substr(0, pos);
                        buffer.erase(0, pos + 1); // Remove the processed part including \r
                        
                        if (!message.empty()) {
                            // Publish the message
                            auto ros_msg = std_msgs::msg::String();
                            ros_msg.data = message;
                            serial_publisher_->publish(ros_msg);
                            
                            RCLCPP_DEBUG(this->get_logger(), "Received message: %s", message.c_str());
                        }
                    }
                } else if (n < 0 && errno != EAGAIN) {
                    RCLCPP_ERROR(this->get_logger(), "Read error: %s", strerror(errno));
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    int serial_fd_;
    std::atomic<bool> running_{true};
    std::thread read_thread_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_publisher_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("motorcpp"), "Starting motor can");
    auto node = std::make_shared<SerialReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}