#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include <GCS_MAVLink/GCS_config.h>
#if HAL_GCS_ENABLED
#include <GCS_MAVLink/GCS_MAVLink.h>
#endif
#include <AP_SerialLED/AP_SerialLED_config.h>
#include <AP_Scripting/AP_Scripting_config.h>





#ifndef HAL_DISPLAY_ENABLED
#define HAL_DISPLAY_ENABLED 1
#endif

#ifndef AP_NOTIFY_DRONECAN_LED_ENABLED
#define AP_NOTIFY_DRONECAN_LED_ENABLED HAL_ENABLE_DRONECAN_DRIVERS
#endif





#ifndef AP_NOTIFY_VRBOARD_LED_ENABLED
#define AP_NOTIFY_VRBOARD_LED_ENABLED 0
#endif

#ifndef AP_NOTIFY_EXTERNALLED_ENABLED
#define AP_NOTIFY_EXTERNALLED_ENABLED 0
#endif





























#ifndef AP_NOTIFY_SCRIPTING_LED_ENABLED
#define AP_NOTIFY_SCRIPTING_LED_ENABLED AP_SCRIPTING_ENABLED
#endif













// Serial LED backends:








#ifndef AP_NOTIFY_TONEALARM_ENABLED
#define AP_NOTIFY_TONEALARM_ENABLED ((defined(HAL_PWM_ALARM) || HAL_DSHOT_ALARM_ENABLED))
#endif
