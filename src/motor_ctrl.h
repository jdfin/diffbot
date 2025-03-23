#pragma once

#include "motor.h"
#include "encoder.h"


class MotorCtrl
{
    public:

        MotorCtrl(const char *gpio_dev_name, unsigned dir_pin,
                  const char *pwm_chip_name, unsigned pwm_num, bool pwm_rev,
                  unsigned enc_a_pin, unsigned enc_b_pin, unsigned enc_cpr,
                  int verbosity=0);

        ~MotorCtrl();

        void speed_rps(double tgt_rps, uint64_t time_ns, double& now_rad, double& now_rps);

        void speed_rps(double tgt_rps, uint64_t now_ns=0)
        {
            double dummy0, dummy1;
            speed_rps(tgt_rps, now_ns, dummy0, dummy1);
        }

        void stop()
        {
            speed_rps(0.0);
        }

        int32_t count()
        {
            return _encoder.count();
        }

        double kp() const { return _kp; }
        double ki() const { return _ki; }
        double kd() const { return _kd; }

        void kp(double kp) { _kp = kp; }
        void ki(double ki) { _ki = ki; }
        void kd(double kd) { _kd = kd; }

    private:

        Motor _motor;

        Encoder _encoder;

        // defaults
        static constexpr double kp_default = 30.0;
        static constexpr double ki_default = 0.0;
        static constexpr double kd_default = 5.0;

        uint64_t _prv_ns;
        int32_t _prv_cnt;
        double _prv_err_rps;
        int _prv_mot;

        double _kp, _ki, _kd;

        int _verbosity;

}; // class MotorCtrl
