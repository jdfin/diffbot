
#include <cstdio>
#include <cstdint>
#include <climits>
#include <cmath> // M_PI

#include "motor.h"
#include "encoder.h"
#include "util.h"
#include "motor_ctrl.h"


MotorCtrl::MotorCtrl(const char *gpio_dev_name, unsigned dir_pin,
                     const char *pwm_chip_name, unsigned pwm_num, bool pwm_rev,
                     unsigned enc_a_pin, unsigned enc_b_pin, unsigned enc_cpr,
                     int verbosity) :
    _motor(gpio_dev_name, dir_pin, pwm_chip_name, pwm_num, pwm_rev, verbosity),
    _encoder(gpio_dev_name, enc_a_pin, enc_b_pin, enc_cpr, verbosity),
    _prv_ns(UINT64_MAX),
    _prv_cnt(INT32_MAX),
    _prv_err_rps(0),
    _prv_mot(0),
    _kp(kp_default),
    _ki(ki_default),
    _kd(kd_default),
    _verbosity(verbosity)
{
    if (verbosity >= 1)
        printf("kp=%f ki=%f kd=%f\n", _kp, _ki, _kd);
}


MotorCtrl::~MotorCtrl()
{
}


void MotorCtrl::speed_rps(double tgt_rps, uint64_t now_ns, double& now_rad, double& now_rps)
{
    if (now_ns == 0)
        now_ns = gettime_ns();

    // time since last update
    if (_prv_ns == UINT64_MAX)
        _prv_ns = now_ns;

    // current position in radians
    int32_t now_cnt = _encoder.count();
    now_rad = (now_cnt * 2.0 * M_PI) / _encoder.counts_per_rev();

    // current speed in rad/sec
    if (_prv_cnt == INT32_MAX)
        _prv_cnt = now_cnt;
    int32_t delta_cnt = now_cnt - _prv_cnt;
    double delta_rad = (delta_cnt * 2.0 * M_PI) / _encoder.counts_per_rev();
    if (now_ns == _prv_ns)
        now_rps = 0.0; // XXX seems wrong, should be previous speed
    else
        now_rps = delta_rad * 1000000000.0 / (now_ns - _prv_ns);

    // run pid to update motor speed
    double err_rps = tgt_rps - now_rps; // err_rps > 0 means go faster please
    double delta_err_rps = err_rps - _prv_err_rps;

    if (_verbosity >= 1) {
        static uint64_t zero_ns = now_ns;
        printf("%0.3f", (now_ns - zero_ns) / 1000000.0);
    }

    if (_verbosity >= 2)
        printf(" %0.4f %0.4f", err_rps, delta_err_rps);

    int now_mot = _prv_mot + round(_kp * err_rps + _kd * delta_err_rps);
    if (now_mot > Motor::speed_max)
        now_mot = Motor::speed_max;
    if (now_mot < -Motor::speed_max)
        now_mot = -Motor::speed_max;

    _motor.speed(now_mot);

    if (_verbosity >= 1)
        printf(" %0.4f %0.4f %d\n", tgt_rps, now_rps, now_mot);

    _prv_ns = now_ns;
    _prv_cnt = now_cnt;
    _prv_err_rps = err_rps;
    _prv_mot = now_mot;
}
