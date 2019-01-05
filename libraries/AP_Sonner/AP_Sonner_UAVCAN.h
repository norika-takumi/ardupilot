#pragma once

#include "AP_Sonner_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

class WaterDepthCb;

class AP_Sonner_UAVCAN : public AP_Sonner_Backend {
public:
    AP_Sonner_UAVCAN(AP_Sonner &sonner);

    void update() override;

    inline void register_sensor() {
        _instance = _frontend.register_sensor();
    }

    static bool subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_Sonner_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new);
    static AP_Sonner_Backend* probe(AP_Sonner &sonner);

    static void handle_WaterDepth(AP_UAVCAN* ap_uavcan, uint8_t node_id, const WaterDepthCb &cb);


private:
  //  static bool subscribed_msgs;

    static bool take_registry();
    static void give_registry();

    uint8_t _instance;

    bool new_WaterDepth;
    uint8_t _WaterDepth;
    uint64_t _last_timestamp;


    HAL_Semaphore _sem_sonner;

    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;

    // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        AP_Sonner_UAVCAN* driver;
    } _detected_modules[SONNER_MAX_DRIVERS];

    static HAL_Semaphore _sem_registry;
};
