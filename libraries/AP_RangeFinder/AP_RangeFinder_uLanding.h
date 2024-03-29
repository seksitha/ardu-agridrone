#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_uLanding : public AP_RangeFinder_Backend
{

public:
    // constructor
	AP_RangeFinder_uLanding(RangeFinder::RangeFinder_State &_state,
                            AP_RangeFinder_Params &_params,
                            uint8_t serial_instance);

    // static detection function
    static bool detect(uint8_t serial_instance);

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
    // detect uLanding Firmware Version
    bool detect_version(void);

    // get a reading
    bool get_reading(uint16_t &reading_cm);

    AP_HAL::UARTDriver *uart;
    uint8_t  _linebuf[6];
    uint8_t  _linebuf_len;
    bool     _version_known;
    uint8_t  _header;
    uint8_t  _version;
};
