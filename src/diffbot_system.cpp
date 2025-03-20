
#include <iostream>
using std::cout;
using std::endl;

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
    right_enc_a_pin_(-1), right_enc_b_pin_(-1), right_enc_cpr_(-1)
{
}


DiffbotSystem::~DiffbotSystem()
{
}


CallbackReturn DiffbotSystem::on_init(const HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

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

    cout << "info.joints:" << endl;
    if (info.joints.size() > 0) {
        for (unsigned i = 0; i < info.joints.size(); i++)
            cout << "  " << i << ":" << " name=" << info.joints[i].name << " type=" << info.joints[i].type << endl;
    } else {
        cout << "  (none)" << endl;
    }

    return CallbackReturn::SUCCESS;
}


CallbackReturn DiffbotSystem::on_configure(const State& /*previous_state*/)
{

// Write the on_configure method where you usually setup the
// communication to the hardware and set everything up so that the
// hardware can be activated.

    return CallbackReturn::ERROR;
}


// Implement on_cleanup method, which does the opposite of on_configure.


CallbackReturn DiffbotSystem::on_activate(const State& /*previous_state*/)
{

// Implement the on_activate method where hardware “power” is enabled.

    return CallbackReturn::ERROR;
}

CallbackReturn DiffbotSystem::on_deactivate(const State& /*previous_state*/)
{

// Implement the on_deactivate method, which does the opposite of
// on_activate.

    return CallbackReturn::ERROR;
}

return_type DiffbotSystem::read(const Time& /*time*/, const Duration& /*period*/)
{

// Implement the read method getting the states from the hardware and
// storing them to internal variables defined in
// export_state_interfaces.

    return return_type::ERROR;
}

return_type DiffbotSystem::write(const Time& /*time*/, const Duration& /*period*/)
{

// Implement write method that commands the hardware based on the values
// stored in internal variables defined in export_command_interfaces.

    return return_type::ERROR;
}

}; // namespace diffbot


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(diffbot::DiffbotSystem, SystemInterface)
