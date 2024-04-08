#include "Sub.h"

// update terrain data
void Sub::terrain_update()
{

    terrain.update();

    // tell the rangefinder our height, so it can go into power saving
    // mode if available

    float height;
    if (terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    }


}

#if HAL_LOGGING_ENABLED
// log terrain data - should be called at 1hz
void Sub::terrain_logging()
{

    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }

}
#endif  // HAL_LOGGING_ENABLED

