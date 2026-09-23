#include "AP_DAL_Compass.h"

#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

AP_DAL_Compass::AP_DAL_Compass()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RMGI); i++) {
        _RMGI[i].instance = i;
    }
}

void AP_DAL_Compass::start_frame()
{
    const auto &compass = AP::compass();

#if AP_CPU_DIAGNOSTICS_ENABLED
    const uint32_t cpu_e3_header_start_us = AP_HAL::micros();
#endif
    const log_RMGH old = _RMGH;
    _RMGH.available = compass.available();
    _RMGH.count = compass.get_count();
    _RMGH.auto_declination_enabled = compass.auto_declination_enabled();
    _RMGH.declination = compass.get_declination();
    _RMGH.first_usable = compass.get_first_usable();
    _RMGH.learn_offsets_enabled = compass.learn_offsets_enabled();
#if AP_CPU_DIAGNOSTICS_ENABLED
    const uint32_t cpu_e3_num_enabled_start_us = AP_HAL::micros();
#endif
    _RMGH.num_enabled = compass.get_num_enabled();
#if AP_CPU_DIAGNOSTICS_ENABLED
    _cpu_e3_num_enabled_us = AP_HAL::micros() - cpu_e3_num_enabled_start_us;
    const uint32_t cpu_e3_consistent_start_us = AP_HAL::micros();
#endif
#if AP_DAL_COMPASS_CONSISTENCY_CACHE_ENABLED
    bool update_consistency = !_consistency_cache_valid ||
                              _RMGH.count != old.count ||
                              _RMGH.first_usable != old.first_usable;
    for (uint8_t i = 0; i < _RMGH.count; i++) {
        if (_consistency_cache_valid &&
            (i >= old.count ||
             compass.use_for_yaw(i) != _RMGI[i].use_for_yaw ||
             compass.last_update_usec(i) != _RMGI[i].last_update_usec ||
             compass.get_field(i) != _RMGI[i].field)) {
            update_consistency = true;
        }
    }
    if (update_consistency) {
        _RMGH.consistent = compass.consistent();
        _consistency_cache_valid = true;
    }
#else
    _RMGH.consistent = compass.consistent();
#endif
#if AP_CPU_DIAGNOSTICS_ENABLED
    _cpu_e3_consistent_us = AP_HAL::micros() - cpu_e3_consistent_start_us;
#endif

#if AP_CPU_DIAGNOSTICS_ENABLED
    _cpu_e3_header_us = AP_HAL::micros() - cpu_e3_header_start_us;
    const uint32_t cpu_e3_header_log_start_us = AP_HAL::micros();
#endif
    WRITE_REPLAY_BLOCK_IFCHANGED(RMGH, _RMGH, old);
#if AP_CPU_DIAGNOSTICS_ENABLED
    _cpu_e3_log_us = AP_HAL::micros() - cpu_e3_header_log_start_us;
#endif

    for (uint8_t i=0; i<_RMGH.count; i++) {
#if AP_CPU_DIAGNOSTICS_ENABLED
        const uint32_t cpu_e3_instance_start_us = AP_HAL::micros();
#endif
        log_RMGI &RMGI = _RMGI[i];
        const log_RMGI old_RMGI = RMGI;
        RMGI.use_for_yaw = compass.use_for_yaw(i);
        RMGI.healthy = compass.healthy(i);
        RMGI.offsets = compass.get_offsets(i);
        RMGI.have_scale_factor = compass.have_scale_factor(i);
        RMGI.last_update_usec = compass.last_update_usec(i);
        RMGI.field = compass.get_field(i);

#if AP_CPU_DIAGNOSTICS_ENABLED
        _cpu_e3_instances_us += AP_HAL::micros() - cpu_e3_instance_start_us;
        const uint32_t cpu_e3_instance_log_start_us = AP_HAL::micros();
#endif
        WRITE_REPLAY_BLOCK_IFCHANGED(RMGI, RMGI, old_RMGI);
#if AP_CPU_DIAGNOSTICS_ENABLED
        _cpu_e3_log_us += AP_HAL::micros() - cpu_e3_instance_log_start_us;
#endif
    }
}
