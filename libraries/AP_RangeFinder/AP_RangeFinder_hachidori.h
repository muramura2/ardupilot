/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_InertialSensor_HACHIDORI;

class AP_RangeFinder_HACHIDORI : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_HACHIDORI(RangeFinder &ranger, uint8_t instance,
                             RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);

private:
    void _timer_update(void);

    uint32_t _last_check_ms;
    uint16_t _distance;
    bool _new_distance;
};
