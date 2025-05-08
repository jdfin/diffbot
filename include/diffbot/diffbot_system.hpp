#ifndef DIFFBOT__DIFFBOT_SYSTEM_HPP
#define DIFFBOT__DIFFBOT_SYSTEM_HPP

#include "hardware_interface/system_interface.hpp"


namespace diffbot
{

class DiffbotSystem : public hardware_interface::SystemInterface
{

    public:

        DiffbotSystem();

        virtual ~DiffbotSystem();

        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;

        hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:

        std::string ser_dev_name_;
        int ser_fd_;

        // actual position (radians) and velocity (radians/second)
        double left_rad_;
        double left_rps_;
        double right_rad_;
        double right_rps_;

}; // class DiffbotSystem

}; // namespace diffbot

#endif // DIFFBOT__DIFFBOT_SYSTEM_HPP
