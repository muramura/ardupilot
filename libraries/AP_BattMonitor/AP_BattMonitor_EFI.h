#pragma once

#include "AP_BattMonitor_Backend.h"



class AP_BattMonitor_EFI : public AP_BattMonitor_Backend
{
public:

    // Inherit constructor
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    // update state
    void read(void) override;

    bool has_current(void) const override {
        return true;
    }

    bool has_consumed_energy(void) const override {
        return true;
    }
};
