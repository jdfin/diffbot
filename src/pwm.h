#pragma once

#include <cstdint>
#include <string>

class Pwm {

    public:

        Pwm(const char* chip, int channel);

        virtual ~Pwm();

        void period_ns(uint32_t period_ns);

        uint32_t period_ns();

        void duty_dpct(int duty_dpct);

        int duty_dpct();

        void enable(bool enable=true);

        void disable() { enable(false); }

        bool enabled();

    private:

        std::string _chip;
        int _chan;

        uint32_t _period_ns;
        int _duty_dpct;

        bool _export;

        const std::string _enable_path;
        const std::string _period_path;
        const std::string _duty_path;

        void set(const std::string& path, uint32_t value);

        uint32_t get(const std::string& path);

        bool file_exists(const std::string& path);

        void wait(const std::string& path);
};
