#include <iostream>
using std::cout;
using std::endl;

#include <fstream>
using std::ifstream;
using std::ofstream;

#include <string>
using std::string;
using std::to_string;

#include <stdexcept>
using std::runtime_error;

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "pwm.h"


Pwm::Pwm(const char* chip, int chan) :
    _chip(chip),
    _chan(chan),
    _enable_path("/sys/class/pwm/" + _chip + "/pwm" + to_string(_chan) + "/enable"),
    _period_path("/sys/class/pwm/" + _chip + "/pwm" + to_string(_chan) + "/period"),
    _duty_path("/sys/class/pwm/" + _chip + "/pwm" + to_string(_chan) + "/duty_cycle")
{
    // do the export (and later unexport) only if the device doesn't already exist
    const string dev_path = "/sys/class/pwm/" + _chip + "/pwm" + to_string(_chan);
    _export = !file_exists(dev_path);
    if (_export) {
        const string export_path = "/sys/class/pwm/" + _chip + "/export";
        //cout << export_path << endl;
        set(export_path, _chan);
    }
    disable();
}


Pwm::~Pwm()
{
    disable();
    if (_export) {
        const string unexport_path = "/sys/class/pwm/" + _chip + "/unexport";
        set(unexport_path, _chan);
    }
}


void Pwm::period_ns(uint32_t period_ns)
{
    _period_ns = period_ns;
    set(_period_path, period_ns);
}


uint32_t Pwm::period_ns()
{
    return get(_period_path);
}


// set duty in deci-percent (0.1%, range 0..1000)
void Pwm::duty_dpct(int duty_dpct)
{
    if (duty_dpct < 0 || duty_dpct > 1000)
        throw std::invalid_argument("duty_dpct out of range");

    _duty_dpct = duty_dpct;

    // if done all at 32 bits, max period is about 42 msec
    uint32_t duty_ns = (uint64_t(_period_ns) * _duty_dpct + 500) / 1000;

    set(_duty_path, duty_ns);
}


int Pwm::duty_dpct()
{
    uint32_t duty_ns = get(_duty_path);
    uint32_t per_ns = period_ns();
    return (duty_ns * 1000 + per_ns / 2) / per_ns;
}


void Pwm::enable(bool enable)
{
    set(_enable_path, enable ? 1 : 0);
}


bool Pwm::enabled()
{
    return get(_enable_path);
}


void Pwm::set(const string& path, uint32_t value)
{
    //cout << path << " = " << value << endl;

    wait(path);

    ofstream f(path);
    f << to_string(value) << endl;
}


uint32_t Pwm::get(const std::string& path)
{
    ifstream f(path);
    uint32_t x;
    f >> x; // this assumes line buffering
    return x;
}


bool Pwm::file_exists(const std::string& path)
{
    return access(path.c_str(), F_OK) == 0;
}


void Pwm::wait(const std::string& path)
{
    // wait for file to appear
    const int timeout_ms = 1000;
    const int poll_ms = 10;
    int timeout_cnt = timeout_ms / poll_ms;
    while (!file_exists(path)) {
        if (timeout_cnt-- < 0)
            throw runtime_error(string("ERROR waiting for ") + path);
        usleep(poll_ms * 1000);
    }
}
