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

        std::string gpio_dev_name_;
        std::string pwm_chip_name_;

        int left_dir_pin_;
        int left_pwm_num_;
        bool left_pwm_rev_;
        int left_enc_a_pin_;
        int left_enc_b_pin_;
        int left_enc_cpr_;

        int right_dir_pin_;
        int right_pwm_num_;
        bool right_pwm_rev_;
        int right_enc_a_pin_;
        int right_enc_b_pin_;
        int right_enc_cpr_;

}; // class DiffbotSystem

}; // namespace diffbot

#endif // DIFFBOT__DIFFBOT_SYSTEM_HPP
