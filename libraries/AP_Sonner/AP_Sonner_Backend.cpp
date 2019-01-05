#include "AP_Sonner_Backend.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Sonner_Backend::AP_Sonner_Backend(AP_Sonner &sonner) : 
    _frontend(sonner) 
{
}

void AP_Sonner_Backend::update_healthy_flag(uint8_t instance)
{
    if (instance >= _frontend._num_sensors) {
        return;
    }
    WITH_SEMAPHORE(_sem);

    // consider a sensor as healthy if it has had an update in the
    // last 0.5 seconds and values are non-zero and have changed within the last 2 seconds
    const uint32_t now = AP_HAL::millis();
    _frontend.sensors[instance].healthy =
        (now - _frontend.sensors[instance].last_update_ms < SONNER_TIMEOUT_MS) &&
        (now - _frontend.sensors[instance].last_change_ms < SONNER_DATA_CHANGE_TIMEOUT_MS) &&
        !is_zero(_frontend.sensors[instance].WaterDepth);
}

void AP_Sonner_Backend::backend_update(uint8_t instance)
{
    update();
    update_healthy_flag(instance);
}


/*
  copy latest data to the frontend from a backend
 */
void AP_Sonner_Backend::_copy_to_frontend(uint8_t instance, float WaterDepth)
{
    if (instance >= _frontend._num_sensors) {
        return;
    }
    uint32_t now = AP_HAL::millis();

    // check for changes in data values
    if (!is_equal(_frontend.sensors[instance].WaterDepth, WaterDepth)) {
        _frontend.sensors[instance].last_change_ms = now;
    }

    // update readings
    _frontend.sensors[instance].WaterDepth = WaterDepth;
    _frontend.sensors[instance].last_update_ms = now;
}

static constexpr float FILTER_KOEF = 0.1f;

/* Check that the baro value is valid by using a mean filter. If the
 * value is further than filtrer_range from mean value, it is
 * rejected. 
*/
bool AP_Sonner_Backend::waterdepth_ok(float waterdepth)
{
    
    if (isinf(waterdepth) || isnan(waterdepth)) {
        return false;
    }

    const float range = (float)_frontend.get_filter_range();
    if (range <= 0) {
        return true;
    }

    bool ret = true;
    return ret;
}
