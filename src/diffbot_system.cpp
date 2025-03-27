
#include <cassert>

#include <iostream>
using std::cout;
using std::endl;

#include "motor_ctrl.h"

#include "diffbot/diffbot_system.hpp"

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using hardware_interface::SystemInterface;
using hardware_interface::CallbackReturn;
using hardware_interface::HardwareInfo;
using hardware_interface::return_type;

using rclcpp_lifecycle::State;

using rclcpp::Time;
using rclcpp::Duration;


namespace diffbot
{


DiffbotSystem::DiffbotSystem() :
    gpio_dev_name_(""), pwm_chip_name_(""),
    left_dir_pin_(-1), left_pwm_num_(-1), left_pwm_rev_(false),
    left_enc_a_pin_(-1), left_enc_b_pin_(-1), left_enc_cpr_(-1),
    right_dir_pin_(-1), right_pwm_num_(-1), right_pwm_rev_(false),
    right_enc_a_pin_(-1), right_enc_b_pin_(-1), right_enc_cpr_(-1),
    left_motor_(nullptr), right_motor_(nullptr),
    left_rad_(0.0), left_rps_(0.0),
    right_rad_(0.0), right_rps_(0.0)
{
}


DiffbotSystem::~DiffbotSystem()
{
}


CallbackReturn DiffbotSystem::on_init(const HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    // const info was used to initialize non-const info_
    // info_.hardware_parameters key and value are both std::string

    gpio_dev_name_ = info_.hardware_parameters["gpio_dev_name"];
    pwm_chip_name_ = info_.hardware_parameters["pwm_chip_name"];
    left_dir_pin_ = std::stoi(info_.hardware_parameters["left/dir_pin"]);
    left_pwm_num_ = std::stoi(info_.hardware_parameters["left/pwm_num"]);
    left_pwm_rev_ = (info_.hardware_parameters["left/pwm_rev"] == "true");
    left_enc_a_pin_ = std::stoi(info_.hardware_parameters["left/enc_a_pin"]);
    left_enc_b_pin_ = std::stoi(info_.hardware_parameters["left/enc_b_pin"]);
    left_enc_cpr_ = std::stoi(info_.hardware_parameters["left/enc_cpr"]);
    right_dir_pin_ = std::stoi(info_.hardware_parameters["right/dir_pin"]);
    right_pwm_num_ = std::stoi(info_.hardware_parameters["right/pwm_num"]);
    right_pwm_rev_ = (info_.hardware_parameters["right/pwm_rev"] == "true");
    right_enc_a_pin_ = std::stoi(info_.hardware_parameters["right/enc_a_pin"]);
    right_enc_b_pin_ = std::stoi(info_.hardware_parameters["right/enc_b_pin"]);
    right_enc_cpr_ = std::stoi(info_.hardware_parameters["right/enc_cpr"]);

#if 1
    cout << "info.hardware_parameters:" << endl
        << "  gpio_dev_name = \"" << gpio_dev_name_ << "\"" << endl
        << "  pwm_chip_name = \"" << pwm_chip_name_ << "\"" << endl
        << "  left/dir_pin = " << left_dir_pin_ << endl
        << "  left/pwm_num = " << left_pwm_num_ << endl
        << "  left/pwm_rev = " << (left_pwm_rev_ ? "true" : "false") << endl
        << "  left/enc_a_pin = " << left_enc_a_pin_ << endl
        << "  left/enc_b_pin = " << left_enc_b_pin_ << endl
        << "  left/enc_cpr = " << left_enc_cpr_ << endl
        << "  right/dir_pin = " << right_dir_pin_ << endl
        << "  right/pwm_num = " << right_pwm_num_ << endl
        << "  right/pwm_rev = " << (right_pwm_rev_ ? "true" : "false") << endl
        << "  right/enc_a_pin = " << right_enc_a_pin_ << endl
        << "  right/enc_b_pin = " << right_enc_b_pin_ << endl
        << "  right/enc_cpr = " << right_enc_cpr_ << endl;
#endif

#if 0
    cout << "info.joints:" << endl;
    if (info.joints.size() > 0) {
        for (unsigned i = 0; i < info.joints.size(); i++)
            cout << "  " << i << ":" << " name=" << info.joints[i].name << " type=" << info.joints[i].type << endl;
    } else {
        cout << "  (none)" << endl;
    }
#endif

    return CallbackReturn::SUCCESS;
}


CallbackReturn DiffbotSystem::on_configure(const State& previous_state)
{
    assert(previous_state.label() == "unconfigured");

    // Usually set up the communication to the hardware and set everything up
    // so that the hardware can be activated

    for (const auto & [name, descr] : joint_state_interfaces_)
        set_state(name, 0.0);

    for (const auto & [name, descr] : joint_command_interfaces_)
        set_command(name, 0.0);

    return CallbackReturn::SUCCESS;
}


// on_cleanup method does the opposite of on_configure


CallbackReturn DiffbotSystem::on_activate(const State& previous_state)
{
    assert(previous_state.label() == "inactive");
    assert(left_motor_ == nullptr);
    assert(right_motor_ == nullptr);

    left_motor_ = new MotorCtrl(gpio_dev_name_.c_str(), left_dir_pin_,
                                pwm_chip_name_.c_str(), left_pwm_num_,
                                left_pwm_rev_, left_enc_a_pin_,
                                left_enc_b_pin_, left_enc_cpr_);

    right_motor_ = new MotorCtrl(gpio_dev_name_.c_str(), right_dir_pin_,
                                 pwm_chip_name_.c_str(), right_pwm_num_,
                                 right_pwm_rev_, right_enc_a_pin_,
                                 right_enc_b_pin_, right_enc_cpr_);

    return CallbackReturn::SUCCESS;
}


CallbackReturn DiffbotSystem::on_deactivate(const State& previous_state)
{
    assert(previous_state.label() == "active");
    assert(left_motor_ != nullptr);
    assert(right_motor_ != nullptr);

    delete left_motor_;
    left_motor_ = nullptr;

    delete right_motor_;
    right_motor_ = nullptr;

    return CallbackReturn::SUCCESS;
}


return_type DiffbotSystem::read(const Time&, const Duration&)
{

    set_state("left_wheel_joint/position", left_rad_);
    set_state("left_wheel_joint/velocity", left_rps_);
    set_state("right_wheel_joint/position", right_rad_);
    set_state("right_wheel_joint/velocity", right_rps_);

#if 0
    if (left_rps_ != 0.0 || right_rps_ != 0.0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "read: pos=(%0.1f %0.1f) vel=(%0.2f %0.2f)",
                             left_rad_, right_rad_, left_rps_, right_rps_);
    }
#endif

    return return_type::OK;
}


return_type DiffbotSystem::write(const Time&, const Duration&)
{
    assert(left_motor_ != nullptr);
    assert(right_motor_ != nullptr);

    auto l_rps = get_command("left_wheel_joint/velocity");
    auto r_rps = get_command("right_wheel_joint/velocity");

    left_motor_->speed_rps(l_rps, 0, left_rad_, left_rps_);
    right_motor_->speed_rps(r_rps, 0, right_rad_, right_rps_);

#if 0
    if (l_rps != 0.0 || r_rps != 0.0 || left_rps_ != 0.0 || right_rps_ != 0.0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "write: cmd=(%0.2f %0.2f) act=(%0.2f %0.2f)",
                             l_rps, r_rps, left_rps_, right_rps_);
    }
#endif

    return return_type::OK;
}


}; // namespace diffbot


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffbot::DiffbotSystem, SystemInterface)
