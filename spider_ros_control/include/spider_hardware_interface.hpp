// using hardware_interface::CallbackReturn;
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "spider_motor.hpp"

namespace spider_ros_control {

class SpiderHardwareInterface : public hardware_interface::SystemInterface { 

    public:
        using return_type = hardware_interface::return_type;
        using State = rclcpp_lifecycle::State;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;

        return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;    

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::string serial_port_;
        std::vector<SpiderMotor> motors_;
        int serial_fd_ = 0;

        bool setup_serial_port(const std::string& port, int baud_rate);
        rclcpp::Logger get_logger();        
};

}

PLUGINLIB_EXPORT_CLASS(spider_ros_control::SpiderHardwareInterface, hardware_interface::SystemInterface)