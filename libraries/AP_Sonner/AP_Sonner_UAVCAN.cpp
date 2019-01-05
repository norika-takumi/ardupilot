#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_Sonner_UAVCAN.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/sonner/WaterDepth.hpp>

extern const AP_HAL::HAL& hal;

#define debug_sonner_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { printf(fmt, ##args); }} while (0)

//UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(WaterDepthCb, uavcan::equipment::sonner::WaterDepth);

AP_Sonner_UAVCAN::DetectedModules AP_Sonner_UAVCAN::_detected_modules[] = {0};
HAL_Semaphore AP_Sonner_UAVCAN::_sem_registry;

/*
  constructor - registers instance at top Sonner driver
 */
AP_Sonner_UAVCAN::AP_Sonner_UAVCAN(AP_Sonner &sonner) :
    AP_Sonner_Backend(sonner)
{
  //  subscribed_msgs = false;
}

bool AP_Sonner_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{


    if (ap_uavcan == nullptr) {
        return false;
    }


    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::sonner::WaterDepth, WaterDepthCb> *waterdepth_listener;
    waterdepth_listener = new uavcan::Subscriber<uavcan::equipment::sonner::WaterDepth, WaterDepthCb>(*node);
    // Msg Handler
    const int waterdepth_listener_res = waterdepth_listener->start(WaterDepthCb(ap_uavcan, &handle_WaterDepth));
    if (waterdepth_listener_res < 0) {
        AP_HAL::panic("UAVCAN sonner subscriber start problem\n\r");
        return false;
    }
    return true;

}

bool AP_Sonner_UAVCAN::take_registry()
{
    return _sem_registry.take(HAL_SEMAPHORE_BLOCK_FOREVER);
}

void AP_Sonner_UAVCAN::give_registry()
{
    _sem_registry.give();
}

AP_Sonner_Backend* AP_Sonner_UAVCAN::probe(AP_Sonner &sonner)
{
 
    if (!take_registry()) {
        return nullptr;
    }
    AP_Sonner_UAVCAN* backend = nullptr;
    for (uint8_t i = 0; i < SONNER_MAX_DRIVERS; i++) {
//        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
        if (_detected_modules[i].driver == nullptr) {
            backend = new AP_Sonner_UAVCAN(sonner);
            if (backend == nullptr) {
                debug_sonner_uavcan(2,
                                  _detected_modules[i].ap_uavcan->get_driver_index(),
                                  "Failed register UAVCAN sonner Node %d on Bus %d\n",
                                  _detected_modules[i].node_id,
                                  _detected_modules[i].ap_uavcan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;
                backend->_ap_uavcan = _detected_modules[i].ap_uavcan;
                backend->_node_id = _detected_modules[i].node_id;
                backend->register_sensor();
                if(_detected_modules[i].ap_uavcan != nullptr)
                {
                debug_sonner_uavcan(2,
                                  _detected_modules[i].ap_uavcan->get_driver_index(),
                                  "Registered UAVCAN sonner Node %d on Bus %d\n",
                                  _detected_modules[i].node_id,
                                  _detected_modules[i].ap_uavcan->get_driver_index());
                }
            }
            break;
        }
    }
    give_registry();
    return backend;
}

AP_Sonner_UAVCAN* AP_Sonner_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < SONNER_MAX_DRIVERS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan && 
            _detected_modules[i].node_id == node_id) {
            return _detected_modules[i].driver;
        }
    }
    
    if (create_new) {
        bool already_detected = false;
        //Check if there's an empty spot for possible registeration
        for (uint8_t i = 0; i < SONNER_MAX_DRIVERS; i++) {
            if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
                //Already Detected
                already_detected = true;
                break;
            }
        }
        if (!already_detected) {
            for (uint8_t i = 0; i < SONNER_MAX_DRIVERS; i++) {
                if (_detected_modules[i].ap_uavcan == nullptr) {
                    _detected_modules[i].ap_uavcan = ap_uavcan;
                    _detected_modules[i].node_id = node_id;
                    _detected_modules[i].driver->_ap_uavcan = ap_uavcan;
                    _detected_modules[i].driver->_node_id = node_id;

                    break;
                }
            }
        }
    }

    return nullptr;
}

void AP_Sonner_UAVCAN::handle_WaterDepth(AP_UAVCAN* ap_uavcan, uint8_t node_id, const WaterDepthCb &cb)
{
    if (take_registry()) {


        AP_Sonner_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, true);
        if (driver == nullptr) {
            give_registry();
            return;
        }
        {
            WITH_SEMAPHORE(driver->_sem_sonner);
            driver->_WaterDepth = cb.msg->WaterDepth;
            driver->new_WaterDepth = true;
        }
        give_registry();
    }
}


// Read the sensor
void AP_Sonner_UAVCAN::update(void)
{

    WITH_SEMAPHORE(_sem_sonner);
    if (new_WaterDepth) {
// gcs().send_text(MAV_SEVERITY_WARNING, "AP_Sonner_UAVCAN::update()new_WaterDepth= %d, %d", new_WaterDepth, _WaterDepth);

        _copy_to_frontend(_instance, _WaterDepth);

        new_WaterDepth = false;
    }
}

#endif // HAL_WITH_UAVCAN

