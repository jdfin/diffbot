
#include <stdexcept>
using std::bad_alloc;

#include <cassert>
#include <cinttypes>
#include <cstdio>
#include <gpiod.h>

#include "encoder.h"

// Define this as forward (A "leads" B):
//        +-------+       +-------+       +----
// A      |       |       |       |       |
//    ----+       +-------+       +-------+
//          +-------+       +-------+       +--
// B        |       |       |       |       |
//    ------+       +-------+       +-------+
//
// Define this as reverse (A "follows" B):
//          +-------+       +-------+       +--
// A        |       |       |       |       |
//    ------+       +-------+       +-------+
//        +-------+       +-------+       +----
// B      |       |       |       |       |
//    ----+       +-------+       +-------+
//
// Note that symmetry is not assumed: in measurements, each channel seems
// to be close to 50%, but the stagger between the channels is not "50%".
// This means when measuring the speed, use one channel or the other, and
// to be safe, use the same edge on that channel, e.g. measure speed using
// only the rising edge on channel A.

Encoder::Encoder(const char *gpio_chip_name,
                 unsigned int pin_a, unsigned int pin_b,
                 unsigned counts_per_rev, int verbosity) :
    _pin_a(pin_a),
    _pin_b(pin_b),
    _counts_per_rev(counts_per_rev),
    _request(nullptr),
    _events(nullptr),
    _event_ns(0),
    _event_last_ns(0),
    _last_a(-1),
    _last_b(-1),
    _count(0),
    _inc(0),
    _interval_avg_len(8),
    _interval_avg_ns(0),
    _verbosity(verbosity)
{
    const unsigned int pins[pin_cnt] = {pin_a, pin_b};

    _request = gpio_init_inputs(gpio_chip_name, pins);
    if (_request == nullptr)
        throw bad_alloc();

    _events = gpiod_edge_event_buffer_new(max_events);
    if (_events == nullptr)
        throw bad_alloc();
}


Encoder::~Encoder()
{
    gpiod_edge_event_buffer_free(_events);
    _events = nullptr;

    gpiod_line_request_release(_request);
    _request = nullptr;
}


gpiod_line_request *Encoder::gpio_init_inputs(const char *gpio_chip_name,
                                              const unsigned int pin_nums[pin_cnt],
                                              const gpiod_line_bias bias, // same for all pins
                                              const char *owner)
{
    gpiod_line_request *request = nullptr;
    gpiod_line_config *line_config = nullptr;
    gpiod_line_settings *line_settings = nullptr;
    gpiod_chip *gpio_chip = nullptr;
    gpiod_request_config *request_config = nullptr;

    line_config = gpiod_line_config_new();
    if (line_config == nullptr) {
        printf("ERROR creating gpiod_line_config");
        goto gpio_init_input_error_0;
    }

    line_settings = gpiod_line_settings_new();
    if (line_settings == nullptr) {
        printf("ERROR creating gpiod_line_settings");
        goto gpio_init_input_error_1;
    }

    gpiod_line_settings_set_direction(line_settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(line_settings, GPIOD_LINE_EDGE_BOTH);
    gpiod_line_settings_set_bias(line_settings, bias);
    gpiod_line_settings_set_debounce_period_us(line_settings, 0);
    gpiod_line_settings_set_event_clock(line_settings, GPIOD_LINE_CLOCK_MONOTONIC);

    if (gpiod_line_config_add_line_settings(line_config, pin_nums, pin_cnt, line_settings) != 0) {
        printf("ERROR adding line_settings to line_config");
        goto gpio_init_input_error_2;
    }

    gpio_chip = gpiod_chip_open(gpio_chip_name);
    if (gpio_chip == nullptr) {
        printf("ERROR opening GPIO chip");
        goto gpio_init_input_error_2;
    }

    request_config = gpiod_request_config_new();
    if (request_config == nullptr) {
        printf("ERROR creating gpiod_request_config");
        goto gpio_init_input_error_3;
    }

    if (owner == nullptr)
        owner = "encoder";
    gpiod_request_config_set_consumer(request_config, owner);

    // default is 16 per line
    gpiod_request_config_set_event_buffer_size(request_config, 256);

    request = gpiod_chip_request_lines(gpio_chip, request_config, line_config);
    if (request == nullptr) {
        printf("ERROR creating gpiod_request");
        goto gpio_init_input_error_4;
    }

gpio_init_input_error_4:

    gpiod_request_config_free(request_config);
    request_config = nullptr;

gpio_init_input_error_3:

    gpiod_chip_close(gpio_chip);
    gpio_chip = nullptr;

gpio_init_input_error_2:

    gpiod_line_settings_free(line_settings);
    line_settings = nullptr;

gpio_init_input_error_1:

    gpiod_line_config_free(line_config);
    line_config = nullptr;

gpio_init_input_error_0:

    return request;
}


void Encoder::process_events()
{
    while (gpiod_line_request_wait_edge_events(_request, 0) == 1) {

        int num_events = gpiod_line_request_read_edge_events(_request, _events, max_events);
        assert(num_events > 0);

        for (unsigned i = 0; i < gpiod_edge_event_buffer_get_num_events(_events); i++) {
            // this returns a pointer into events
            gpiod_edge_event *event = gpiod_edge_event_buffer_get_event(_events, i);

            unsigned int pin_num = gpiod_edge_event_get_line_offset(event);

            // for the sake of brevity
            const gpiod_edge_event_type rise = GPIOD_EDGE_EVENT_RISING_EDGE;
            const gpiod_edge_event_type fall = GPIOD_EDGE_EVENT_FALLING_EDGE;

            gpiod_edge_event_type edge = gpiod_edge_event_get_event_type(event);

            assert(pin_num == _pin_a || pin_num == _pin_b);
            assert(edge == rise || edge == fall);

            // don't count until we've seen an edge on each
            if (_last_a == -1 || _last_b == -1) {
                if (pin_num == _pin_a) {
                    _last_a = (edge == rise ? 1 : 0);
                } else {
                    assert(pin_num == _pin_b);
                    _last_b = (edge == rise ? 1 : 0);
                }
            } else {
                assert(_last_a == 0 || _last_a == 1);
                assert(_last_b == 0 || _last_b == 1);

                int inc = 0;

                if (_last_a == 0 && _last_b == 0) {
                    if (pin_num == _pin_a && edge == rise) {
                        inc = +1;
                        _last_a = 1;
                    } else if (pin_num == _pin_b && edge == rise) {
                        inc = -1;
                        _last_b = 1;
                    }
                } else if (_last_a == 0 && _last_b == 1) {
                    if (pin_num == _pin_a && edge == rise) {
                        inc = -1;
                        _last_a = 1;
                    } else if (pin_num == _pin_b && edge == fall) {
                        inc = +1;
                        _last_b = 0;
                    }
                } else if (_last_a == 1 && _last_b == 0) {
                    if (pin_num == _pin_a && edge == fall) {
                        inc = -1;
                        _last_a = 0;
                    } else if (pin_num == _pin_b && edge == rise) {
                        inc = +1;
                        _last_b = 1;
                    }
                } else if (_last_a == 1 && _last_b == 1) {
                    if (pin_num == _pin_a && edge == fall) {
                        inc = +1;
                        _last_a = 0;
                    } else if (pin_num == _pin_b && edge == fall) {
                        inc = -1;
                        _last_b = 0;
                    }
                }

                if (inc == 0) {
                    // unexpected edge
                    if (_verbosity >= 1)
                        printf("last_a=%d last_b=%d: pin %d went %s\n", _last_a, _last_b, pin_num, edge == rise ? "hi" : "lo");
                } else {
                    assert(inc == 1 || inc == -1);
                    _inc = inc;
                    _count += inc;
                    _event_ns = gpiod_edge_event_get_timestamp_ns(event);

                    // measure interval using rising edge of A
                    if (pin_num == _pin_a && edge == rise) {
                        if (_event_last_ns != 0) {
                            uint64_t interval_ns = _event_ns - _event_last_ns;
                            if (_interval_avg_ns == 0)
                                _interval_avg_ns = interval_ns;
                            else
                                _interval_avg_ns = (_interval_avg_ns * (_interval_avg_len - 1) + interval_ns) / _interval_avg_len;
                        }
                        _event_last_ns = _event_ns;
                    }
                }

            } // if (_last_a == -1 || _last_b == -1) {

        } // for (unsigned i...)

    } // while (gpiod_line_request_wait_edge_events...)

} // void Encoder::process_events()
