
#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_MAVLINK_RADIO_MASKED_ENABLED

#include "AP_RCProtocol.h"


class AP_RCProtocol_MAVLinkRadioMasked : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    // update from mavlink messages
    void update_radio_rc_channels_masked(const mavlink_radio_rc_channels_masked_t* packet) override;
};

#endif // AP_RCPROTOCOL_MAVLINK_RADIO_MASKED_ENABLED

