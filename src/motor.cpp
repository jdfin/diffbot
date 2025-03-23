
#include <stdexcept>
using std::bad_alloc;
using std::runtime_error;
using std::invalid_argument;

#include <unistd.h>
#include <gpiod.h>
#include "pwm.h"
#include "motor.h"


Motor::Motor(const char* gpio_dev_name, unsigned dir_pin,
             const char* pwm_chip_name, unsigned pwm_num,
             bool reverse, int verbosity) :
    _request(nullptr),
    _dir_pin(dir_pin),
    _dir(0),
    _dir_fwd(reverse ? 1 : 0),
    _pwm(pwm_chip_name, pwm_num),
    _verbosity(verbosity)
{
    const unsigned int pin = dir_pin;
    const gpiod_line_value val = GPIOD_LINE_VALUE_INACTIVE;

    _request = gpio_init_outputs(gpio_dev_name, 1, &pin, &val);
    if (_request == nullptr)
        throw bad_alloc();

    // Odd, one or both of these delays seems to be needed.
    // 10 msec is usually but not always enough.
    // Have not seen 15 msec not be enough.
    usleep(20000);
    _pwm.period_ns(40000);  // 25 KHz
    usleep(20000);
    _pwm.duty_dpct(0);
    _pwm.enable();
}


Motor::~Motor()
{
    if (_request != nullptr) {
        set_dir(_dir_fwd);
        gpiod_line_request_release(_request);
        _request = nullptr;
    }
}


void Motor::speed(int spd)
{
    if (spd < -speed_max || spd > speed_max)
        throw invalid_argument("speed invalid");

    if (spd >= 0) {
        set_dir(_dir_fwd);
    } else {
        set_dir(1 - _dir_fwd);
        spd = -spd;
    }

    _pwm.duty_dpct(spd);
}


void Motor::set_dir(int dir)
{
    if (dir != 0 && dir != 1)
        throw invalid_argument("direction invalid");

    _dir = dir;

    const gpiod_line_value val = (_dir == 0 ? GPIOD_LINE_VALUE_INACTIVE : GPIOD_LINE_VALUE_ACTIVE);

    if (gpiod_line_request_set_value(_request, _dir_pin, val) != 0)
        throw runtime_error("ERROR setting GPIO line");
}


gpiod_line_request *Motor::gpio_init_outputs(const char *gpio_dev_name,
                                             int pin_cnt,
                                             const unsigned int *pin_nums,
                                             const gpiod_line_value *line_values,
                                             gpiod_line_drive line_drive,
                                             const char *owner)
{
    struct gpiod_line_request *request = nullptr;
    gpiod_line_config *line_config = nullptr;
    gpiod_line_settings *line_settings = nullptr;
    gpiod_chip *gpio_chip = nullptr;
    gpiod_request_config *request_config = nullptr;

    line_config = gpiod_line_config_new();
    if (line_config == nullptr) {
        printf("ERROR creating gpiod_line_config\n");
        goto gpio_init_output_error_0;
    }

    line_settings = gpiod_line_settings_new();
    if (line_settings == nullptr) {
        printf("ERROR creating gpiod_line_settings\n");
        goto gpio_init_output_error_1;
    }

    gpiod_line_settings_set_direction(line_settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_drive(line_settings, line_drive);

    if (gpiod_line_config_add_line_settings(line_config, pin_nums, pin_cnt, line_settings) != 0) {
        printf("ERROR adding line_settings to line_config\n");
        goto gpio_init_output_error_2;
    }

    if (gpiod_line_config_set_output_values(line_config, line_values, pin_cnt) != 0) {
        printf("ERROR setting initial values in line_config\n");
        goto gpio_init_output_error_2;
    }

    gpio_chip = gpiod_chip_open(gpio_dev_name);
    if (gpio_chip == nullptr) {
        printf("ERROR opening GPIO chip\n");
        goto gpio_init_output_error_2;
    }

    request_config = gpiod_request_config_new();
    if (request_config == nullptr) {
        printf("ERROR creating gpiod_request_config\n");
        goto gpio_init_output_error_3;
    }

    if (owner == nullptr)
        owner = "motor";

    gpiod_request_config_set_consumer(request_config, owner);

    request = gpiod_chip_request_lines(gpio_chip, request_config, line_config);
    if (request == nullptr) {
        printf("ERROR creating gpiod_request\n");
        goto gpio_init_output_error_4;
    }

gpio_init_output_error_4:

    gpiod_request_config_free(request_config);
    request_config = nullptr;

gpio_init_output_error_3:

    gpiod_chip_close(gpio_chip);
    gpio_chip = nullptr;

gpio_init_output_error_2:

    gpiod_line_settings_free(line_settings);
    line_settings = nullptr;

gpio_init_output_error_1:

    gpiod_line_config_free(line_config);
    line_config = nullptr;

gpio_init_output_error_0:

    return request;
}
