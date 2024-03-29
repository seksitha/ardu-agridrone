/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "AP_RCProtocol.h"
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Logger/AP_Logger.h>

AP_RCProtocol_Backend::AP_RCProtocol_Backend(AP_RCProtocol &_frontend) :
    frontend(_frontend),
    rc_input_count(0),
    last_rc_input_count(0),
    _num_channels(0)
{}

bool AP_RCProtocol_Backend::new_input()
{
    bool ret = rc_input_count != last_rc_input_count;
    if (ret) {
        last_rc_input_count = rc_input_count;
    }
    return ret;
}

uint8_t AP_RCProtocol_Backend::num_channels()
{
    return _num_channels;
}

uint16_t AP_RCProtocol_Backend::read(uint8_t chan)
{
    return _pwm_values[chan];
}

void AP_RCProtocol_Backend::read(uint16_t *pwm, uint8_t n)
{
    if (n >= MAX_RCIN_CHANNELS) {
        n = MAX_RCIN_CHANNELS;
    }
    memcpy(pwm, _pwm_values, n*sizeof(pwm[0]));
}

/*
  provide input from a backend
 */
void AP_RCProtocol_Backend::add_input(uint8_t num_values, uint16_t *values, bool in_failsafe, int16_t _rssi)
{
    num_values = MIN(num_values, MAX_RCIN_CHANNELS);
    memcpy(_pwm_values, values, num_values*sizeof(uint16_t));
    _num_channels = num_values;
    rc_frame_count++;
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware)
    if (rc().ignore_rc_failsafe()) {
        in_failsafe = false;
    }
#endif
    if (!in_failsafe) {
        rc_input_count++;
    }
    rssi = _rssi;
}

