
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


DiffbotSystem::DiffbotSystem()
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

// Implement on_init method. Here, you should initialize all member
// variables and process the parameters from the info argument. In the
// first line usually the parents on_init is called to process standard
// values, like name. This is done using SystemInterface::on_init(info).
// If all required parameters are set and valid and everything works
// fine return SUCCESS or return ERROR otherwise.

    return CallbackReturn::ERROR;
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
