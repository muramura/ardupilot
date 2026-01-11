#include "AP_RCProtocol_config.h"
#include "GCS_MAVLink/GCS.h"

#if AP_RCPROTOCOL_MAVLINK_RADIO_MASKED_ENABLED

#include "AP_RCProtocol_MAVLinkRadioMasked.h"

void AP_RCProtocol_MAVLinkRadioMasked::update_radio_rc_channels_masked(
    const mavlink_radio_rc_channels_masked_t* packet)
{
    uint16_t rc_chan[MAX_RCIN_CHANNELS];

    // Ignore empty packets
    if (packet->count < 1) {
        return;
    }

    // Initialize buffer: UINT16_MAX means "no update" for this channel
    memset(rc_chan, 0xFF, sizeof(rc_chan));

    // COUNT represents the number of physical RC channels (CH1..CHCOUNT)
    const uint8_t count = (packet->count < MAX_RCIN_CHANNELS) ? packet->count : MAX_RCIN_CHANNELS;
    const uint32_t mask = packet->active_mask;
    uint8_t index = 0;

    // Iterate over physical channel numbers
    for (uint8_t ch = 0; ch < count; ch++) {

        // Skip channels not selected by the active mask
        if ((mask & (1U << ch)) == 0) {
            continue;
        }

        // Packed format:
        // channels[] contains values only for channels whose bits are set
        // in active_mask, ordered by ascending channel number.
        rc_chan[ch] = ((int32_t)packet->channels[index++] * 5) / 32 + 1500;

    }

    // GCS_SEND_TEXT(
    // MAV_SEVERITY_NOTICE,
    // "### count=%u active_mask=0x%08lX rc1 %u rc3 %u rc16 %u",
    // count,
    // (unsigned long)mask,
    // rc_chan[0],
    // rc_chan[2],
    // rc_chan[15]);

    // Bit 0: FAILSAFE (1 = link lost / failsafe)
    const bool failsafe = packet->flags & RADIO_RC_CHANNELS_FLAGS_FAILSAFE;

    // Feed the RC input into the RCProtocol layer
    add_input(count, rc_chan, failsafe);
}

#endif // AP_RCPROTOCOL_MAVLINK_RADIO_MASKED_ENABLED
