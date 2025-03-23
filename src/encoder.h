#pragma once

#include <cassert>
#include <cstdint>
#include <gpiod.h>


class Encoder {

    public:

        Encoder(const char *gpio_chip_name,
                unsigned int pin_a, unsigned int pin_b,
                unsigned counts_per_rev, int verbosity=1);

        ~Encoder();

        void update()
        {
            process_events();
        }

        int32_t count()
        {
            process_events();
            return _count;
        }

        unsigned counts_per_rev() const
        {
            return _counts_per_rev;
        }

        uint64_t interval_ns(bool avg=true) const
        {
            if (_event_last_ns == 0)
                return 0;

            assert(_event_ns != 0);

            if (avg)
                return _interval_avg_ns;
            else
                return _event_ns - _event_last_ns;
        }

        // count rate in ticks/sec
        int32_t rate_tps(bool avg=true) const
        {
            uint64_t int_ns = interval_ns(avg);
            if (int_ns == 0)
                return 0; // no intervals yet

            if (_inc == 0)
                return 0; // haven't moved yet

            assert(_inc == -1 || _inc == +1);

            // 10^9 nsec    interval     4 ticks    ticks
            // --------- x ----------- x -------- = -----
            //    sec      int_ns nsec   interval    sec

            uint64_t tps = 4000000000ull / int_ns;

            return _inc * tps;
        }

    private:

        static const int max_events = 1000;

        static const int pin_cnt = 2;

        unsigned int _pin_a;
        unsigned int _pin_b;

        unsigned _counts_per_rev;

        gpiod_line_request *_request;

        gpiod_edge_event_buffer *_events;

        uint64_t _event_ns;

        uint64_t _event_last_ns;

        int _last_a;
        int _last_b;

        int32_t _count;

        int _inc; // last increment, to keep track of direction

        int _interval_avg_len; // moving average length

        uint64_t _interval_avg_ns; // moving average

        int _verbosity;

        gpiod_line_request *gpio_init_inputs(const char *gpio_chip_name,
                                             const unsigned int pin_nums[pin_cnt],
                                             const gpiod_line_bias bias=GPIOD_LINE_BIAS_DISABLED,
                                             const char *owner=nullptr);

        void process_events();

}; // class Encoder
