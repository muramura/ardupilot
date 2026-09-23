#pragma once

#include <AP_Common/AP_CPU_Diagnostics_config.h>
#include <AP_Logger/LogStructure.h>

#include <AP_Compass/AP_Compass.h>

#ifndef AP_DAL_COMPASS_CONSISTENCY_CACHE_ENABLED
#define AP_DAL_COMPASS_CONSISTENCY_CACHE_ENABLED 0
#endif

class AP_DAL_Compass {
public:

    // Compass-like methods:
    bool use_for_yaw(uint8_t i) const {
        return _RMGI[i].use_for_yaw;
    }

    bool healthy(uint8_t i) const {
        return _RMGI[i].healthy;
    }

    const Vector3f &get_offsets(uint8_t i) const {
        return _RMGI[i].offsets;
    }

    bool have_scale_factor(uint8_t i) const {
        return _RMGI[i].have_scale_factor;
    }

    bool auto_declination_enabled() const {
        return _RMGH.auto_declination_enabled;
    }

    uint8_t get_count() const {
        return _RMGH.count;
    }

    float get_declination() const {
        return _RMGH.declination;
    }

    bool available() const {
        return _RMGH.available;
    }

    // return the number of enabled sensors
    uint8_t get_num_enabled(void) const { return _RMGH.num_enabled; }

    // learn offsets accessor
    bool learn_offsets_enabled() const { return _RMGH.learn_offsets_enabled; }

    // return last update time in microseconds
    uint32_t last_update_usec(uint8_t i) const { return _RMGI[i].last_update_usec; }

    /// Return the current field as a Vector3f in milligauss
    const Vector3f &get_field(uint8_t i) const { return _RMGI[i].field; }

    // check if the compasses are pointing in the same direction
    bool consistent() const { return _RMGH.consistent; }

    // returns first usable compass
    uint8_t get_first_usable() const { return _RMGH.first_usable; }

    // AP_DAL methods:
    AP_DAL_Compass();

    void start_frame();
#if AP_CPU_DIAGNOSTICS_ENABLED
    void reset_cpu_e3_timing() {
        _cpu_e3_header_us = 0;
        _cpu_e3_instances_us = 0;
        _cpu_e3_log_us = 0;
        _cpu_e3_consistent_us = 0;
        _cpu_e3_num_enabled_us = 0;
    }
    uint32_t get_cpu_e3_header_us() const { return _cpu_e3_header_us; }
    uint32_t get_cpu_e3_instances_us() const { return _cpu_e3_instances_us; }
    uint32_t get_cpu_e3_log_us() const { return _cpu_e3_log_us; }
    uint32_t get_cpu_e3_consistent_us() const { return _cpu_e3_consistent_us; }
    uint32_t get_cpu_e3_num_enabled_us() const { return _cpu_e3_num_enabled_us; }
#endif

    void handle_message(const log_RMGH &msg) {
        _RMGH = msg;
    }
    void handle_message(const log_RMGI &msg) {
        _RMGI[msg.instance] = msg;
    }

private:

    struct log_RMGH _RMGH;
    struct log_RMGI _RMGI[COMPASS_MAX_INSTANCES];
#if AP_DAL_COMPASS_CONSISTENCY_CACHE_ENABLED
    bool _consistency_cache_valid{};
#endif
#if AP_CPU_DIAGNOSTICS_ENABLED
    uint32_t _cpu_e3_header_us;
    uint32_t _cpu_e3_instances_us;
    uint32_t _cpu_e3_log_us;
    uint32_t _cpu_e3_consistent_us;
    uint32_t _cpu_e3_num_enabled_us;
#endif
};
