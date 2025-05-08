
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <cassert>

#include <iostream>
using std::cout;
using std::endl;

#include "rosbot/rosbot_system.hpp"

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


namespace rosbot
{


RosBotSystem::RosBotSystem() :
    ser_dev_name_(""), ser_fd_(-1),
    left_rad_(0.0), left_rps_(0.0),
    right_rad_(0.0), right_rps_(0.0)
{
}


RosBotSystem::~RosBotSystem()
{
}


CallbackReturn RosBotSystem::on_init(const HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    // const info was used to initialize non-const info_
    // info_.hardware_parameters key and value are both std::string

    ser_dev_name_ = info_.hardware_parameters["ser_dev_name"];

#if 1
    cout << "info.hardware_parameters:" << endl
        << "  ser_dev_name = \"" << ser_dev_name_ << "\"" << endl;
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


CallbackReturn RosBotSystem::on_configure(const State& previous_state)
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


CallbackReturn RosBotSystem::on_activate(const State& previous_state)
{
    assert(previous_state.label() == "inactive");

    assert(ser_fd_ == -1);

    ser_fd_ = open(ser_dev_name_.c_str(), O_RDWR);

#if 0
    cout << "\"" << ser_dev_name_ << "\" opened" << endl;
#endif

    assert(ser_fd_ != -1);

    struct termios tc;

    assert(tcgetattr(ser_fd_, &tc) == 0);

    cfmakeraw(&tc);
    cfsetspeed(&tc, B115200);

    assert(tcsetattr(ser_fd_, TCSANOW, &tc) == 0);

    return CallbackReturn::SUCCESS;
}


CallbackReturn RosBotSystem::on_deactivate(const State& previous_state)
{
    assert(previous_state.label() == "active");

    assert(ser_fd_ != -1);
    close(ser_fd_);
    ser_fd_ = -1;

    return CallbackReturn::SUCCESS;
}


return_type RosBotSystem::read(const Time&, const Duration&)
{
    //cout << "RosBotSystem::read: enter" << endl;

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

    //cout << "RosBotSystem::read: return OK" << endl;

    return return_type::OK;
}


return_type RosBotSystem::write(const Time&, const Duration&)
{
    //cout << "RosBotSystem::write: enter" << endl;

    assert(ser_fd_ != -1);

    auto l_rps = get_command("left_wheel_joint/velocity");
    auto r_rps = get_command("right_wheel_joint/velocity");

    // convert radians/second to steps/second
    const int32_t steps_per_rev = 200 * 16;
    const double steps_per_rad = steps_per_rev / (2 * M_PI);
    int32_t l_sps = round(l_rps * steps_per_rad);
    int32_t r_sps = round(r_rps * steps_per_rad);

    char buf[80];

    sprintf(buf, "s %d %d", l_sps, r_sps);

    //cout << "RosBotSystem::write: sending \"" << buf << "\"" << endl;
    ::write(ser_fd_, buf, strlen(buf) + 1);

    // get current step counts

    size_t idx = 0;
    while (idx < sizeof(buf)) {
        char c;
        ::read(ser_fd_, &c, 1);
        //printf(" %02x\n", (int)c);
        // messages normally end in \r\n; ignore the \r and end it on the \n
        if (c == '\r')
            continue;
        if (c == '\n')
            c = '\0';
        buf[idx++] = c;
        if (c == '\0')
            break;
    }

    //cout << "RosBotSystem::write: received \"" << buf << "\"" << endl;

    int32_t l_step, r_step;
    if (sscanf(buf, "g %d %d", &l_step, &r_step) == 2) {
        // convert steps to radians
        left_rad_ = l_step / steps_per_rad;
        right_rad_ = r_step / steps_per_rad;
    }

#if 0
    if (l_rps != 0.0 || r_rps != 0.0 || left_rps_ != 0.0 || right_rps_ != 0.0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "write: cmd=(%0.2f %0.2f) act=(%0.2f %0.2f)",
                             l_rps, r_rps, left_rps_, right_rps_);
    }
#endif

    //cout << "RosBotSystem::write: return OK" << endl;

    return return_type::OK;
}


}; // namespace rosbot


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rosbot::RosBotSystem, SystemInterface)
