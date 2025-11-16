// using hardware_interface::CallbackReturn;
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "spider_motor.hpp"

class SpiderHardwareInterface : public hardware_interface::SystemInterface {
    using return_type = hardware_interface::return_type;

    public:    
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;

        return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;    

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::string serial_port_;
        rclcpp::Logger logger_;    
        std::vector<SpiderMotor> motors_;
        int serial_fd_ = 0;

        bool setup_serial_port(const std::string& port, int baud_rate);
};