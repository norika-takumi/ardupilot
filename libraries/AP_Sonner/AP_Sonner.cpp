/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       APM_Baro.cpp - barometer driver
 *
 */
#include "AP_Sonner.h"

#include <utility>
#include <stdio.h>

#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#if HAL_WITH_UAVCAN
#include "AP_Sonner_UAVCAN.h"
#endif


#ifndef HAL_SONNER_FILTER_DEFAULT
 #define HAL_SONNER_FILTER_DEFAULT 0 // turned off by default
#endif


extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Sonner::var_info[] = {
    // NOTE: Index numbers 0 and 1 were for the old integer
    // ground temperature and pressure


    // @Param: PRIMARY
    // @DisplayName: Primary sonner
    // @Description: This selects which sonner will be the primary if multiple sonners are found
    // @Values: 0:FirstSonner,1:2ndSonner,2:3rdSonner
    // @User: Advanced
    AP_GROUPINFO("PRIMARY", 2, AP_Sonner, _primary_sonner, 0),



    AP_GROUPEND
};

// singleton instance
AP_Sonner *AP_Sonner::_instance;

/*
  AP_Sonner constructor
 */
AP_Sonner::AP_Sonner()
{
    _instance = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// calibrate the barometer. This must be called at least once before
// the altitude() or climb_rate() interfaces can be used
void AP_Sonner::calibrate(bool save)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Calibrating sonner");


    // start by assuming all sensors are calibrated (for healthy() test)
    for (uint8_t i=0; i<_num_sensors; i++) {
        sensors[i].calibrated = true;
        sensors[i].alt_ok = true;
    }



    // panic if all sensors are not calibrated
    uint8_t num_calibrated = 0;
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (sensors[i].calibrated) {
            gcs().send_text(MAV_SEVERITY_INFO, "Sonner %u calibration complete", i+1);
            num_calibrated++;
        }
    }
    if (num_calibrated) {
        return;
    }
    AP_HAL::panic("AP_Sonner: all sensors uncalibrated");
}

/*
   update the barometer calibration
   this updates the baro ground calibration to the current values. It
   can be used before arming to keep the baro well calibrated
*/
void AP_Sonner::update_calibration()
{
    const uint32_t now = AP_HAL::millis();
    const bool do_notify = now - _last_notify_ms > 10000;
    if (do_notify) {
        _last_notify_ms = now;
    }
}



bool AP_Sonner::_add_backend(AP_Sonner_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (_num_drivers >= SONNER_MAX_DRIVERS) {
        AP_HAL::panic("Too many sonner drivers");
    }
    drivers[_num_drivers++] = backend;
    return true;
}

/*
  macro to add a backend with check for too many sensors
 We don't try to start more than the maximum allowed
 */
#define ADD_BACKEND(backend) \
    do { _add_backend(backend);     \
       if (_num_drivers == SONNER_MAX_DRIVERS || \
          _num_sensors == SONNER_MAX_INSTANCES) { \
          return; \
       } \
    } while (0)

/*
  initialise the barometer object, loading backend drivers
 */
void AP_Sonner::init(void)
{

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
//    ADD_BACKEND(new AP_Sonner_SITL(*this));
//    return;
#endif

#if HAL_WITH_UAVCAN
    // Detect UAVCAN Modules, try as many times as there are driver slots
    for (uint8_t i = 0; i < SONNER_MAX_DRIVERS; i++) {
        ADD_BACKEND(AP_Sonner_UAVCAN::probe(*this));
    }
#endif



}


bool AP_Sonner::should_df_log() const
{
    DataFlash_Class *instance = DataFlash_Class::instance();
    if (instance == nullptr) {
        return false;
    }
    if (_log_baro_bit == (uint32_t)-1) {
        return false;
    }
//    if (!instance->should_log(_log_sonner_bit)) {
//        return false;
//    }
    return true;
}

/*
  call update on all drivers
 */
void AP_Sonner::update(void)
{
 //gcs().send_text(MAV_SEVERITY_WARNING, "AP_Sonner::update()");


    for (uint8_t i=0; i<_num_drivers; i++) {
        drivers[i]->backend_update(i);
    }

    // choose primary sensor
    if (_primary_sonner >= 0 && _primary_sonner < _num_sensors && healthy(_primary_sonner)) {
        _primary = _primary_sonner;
    } else {
        _primary = 0;
        for (uint8_t i=0; i<_num_sensors; i++) {
            if (healthy(i)) {
                _primary = i;
                break;
            }
        }
    }

    // logging
    if (should_df_log() && !AP::ahrs().have_ekf_logging()) {
 //       DataFlash_Class::instance()->Log_Write_Sonner();
    }
}



/* register a new sensor, claiming a sensor slot. If we are out of
   slots it will panic
*/
uint8_t AP_Sonner::register_sensor(void)
{
    if (_num_sensors >= SONNER_MAX_INSTANCES) {
        AP_HAL::panic("Too many barometers");
    }
    return _num_sensors++;
}


/*
  check if all barometers are healthy
 */
bool AP_Sonner::all_healthy(void) const
{
     for (uint8_t i=0; i<_num_sensors; i++) {
         if (!healthy(i)) {
             return false;
         }
     }
     return _num_sensors > 0;
}


namespace AP {

AP_Sonner &sonner()
{
    return *AP_Sonner::get_instance();
}

};
