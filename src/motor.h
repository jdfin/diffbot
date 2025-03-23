#pragma once

#include <gpiod.h>

#include "pwm.h"

// DC motor controlled by Pololu DRV8835 module with pwm speed

class Motor {

    public:

        Motor(const char* gpio_dev_name, unsigned dir_pin,
              const char* pwm_chip_name, unsigned pwm_num,
              bool reverse=false, int verbosity=0);

        virtual ~Motor();

        static constexpr int speed_max = 1000;

        void speed(int spd);

    private:

        gpiod_line_request *_request;

        const unsigned _dir_pin;

        int _dir; // 0=forward, 1=reverse

        int _dir_fwd;

        Pwm _pwm;

        int _verbosity;

        void set_dir(int dir);

        gpiod_line_request *gpio_init_outputs(const char *gpio_chip_name,
                                              int pin_cnt,
                                              const unsigned int *pin_nums,
                                              const gpiod_line_value *line_values,
                                              gpiod_line_drive line_drive=GPIOD_LINE_DRIVE_PUSH_PULL,
                                              const char *owner=nullptr);
};
