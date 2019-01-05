#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <Filter/Filter.h>
#include <Filter/DerivativeFilter.h>

// maximum number of sensor instances
#define SONNER_MAX_INSTANCES 1

// maximum number of drivers. Note that a single driver can provide
// multiple sensor instances
#define SONNER_MAX_DRIVERS 1

// timeouts for health reporting
#define SONNER_TIMEOUT_MS                 500     // timeout in ms since last successful read
#define SONNER_DATA_CHANGE_TIMEOUT_MS     2000    // timeout in ms since last successful read that involved temperature of pressure changing

class AP_Sonner_Backend;

class AP_Sonner
{
    friend class AP_Sonner_Backend;
    friend class AP_Sonner_SITL; // for access to sensors[]

public:
    AP_Sonner();

    /* Do not allow copies */
    AP_Sonner(const AP_Sonner &other) = delete;
    AP_Sonner &operator=(const AP_Sonner&) = delete;

    // get singleton
    static AP_Sonner *get_instance(void) {
        return _instance;
    }

    // sonner types
    typedef enum {
        SONNER_TYPE_AIR,
        SONNER_TYPE_WATER
    } sonner_type_t;

    // initialise the barometer object, loading backend drivers
    void init(void);

    // update the barometer object, asking backends to push data to
    // the frontend
    void update(void);

    // healthy - returns true if sensor and derived altitude are good
    bool healthy(void) const { return healthy(_primary); }
    bool healthy(uint8_t instance) const { return sensors[instance].healthy && sensors[instance].alt_ok && sensors[instance].calibrated; }

    // check if all baros are healthy - used for SYS_STATUS report
    bool all_healthy(void) const;

    // waterdepth in Pascal. 
    uint8_t get_waterdepth(void) const { return get_waterdepth(_primary); }
    uint8_t get_waterdepth(uint8_t instance) const { return sensors[instance].WaterDepth; }

    int get_num_drivers() { return _num_drivers; }

    // accumulate a reading on sensors. Some backends without their
    // own thread or a timer may need this.
    void accumulate(void);

    // calibrate the barometer. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void calibrate(bool save=true);

    // update the barometer calibration to the current pressure. Can
    // be used for incremental preflight update of baro
    void update_calibration(void);


    // get last time sample was taken (in ms)
    uint32_t get_last_update(void) const { return get_last_update(_primary); }
    uint32_t get_last_update(uint8_t instance) const { return sensors[instance].last_update_ms; }

    // settable parameters
    static const struct AP_Param::GroupInfo var_info[];


    // Set the primary baro
    void set_primary_sonner(uint8_t primary) { _primary_sonner.set_and_save(primary); };


    // HIL variables
    struct {
        float waterdepth;
        uint32_t last_update_ms;
        bool updated:1;
        bool have_alt:1;
        bool have_last_update:1;
    } _hil;

    // register a new sensor, claiming a sensor slot. If we are out of
    // slots it will panic
    uint8_t register_sensor(void);

    // return number of registered sensors
    uint8_t num_instances(void) const { return _num_sensors; }



    uint8_t get_filter_range() const { return _filter_range; }

    // indicate which bit in LOG_BITMASK indicates baro logging enabled
    bool should_df_log() const;

private:
    // singleton
    static AP_Sonner *_instance;
    
    // how many drivers do we have?
    uint8_t _num_drivers;
    AP_Sonner_Backend *drivers[SONNER_MAX_DRIVERS];

    // how many sensors do we have?
    uint8_t _num_sensors;

    // what is the primary sensor at the moment?
    uint8_t _primary;

    uint32_t _log_baro_bit = -1;

    // bitmask values for GND_PROBE_EXT
    enum {
        PROBE_BMP085=(1<<0),
    };
    
    struct sensor {
        sonner_type_t type;                   // 0 for air pressure (default), 1 for water pressure
        uint32_t last_update_ms;        // last update time in ms
        uint32_t last_change_ms;        // last update time in ms that included a change in reading from previous readings
        bool healthy:1;                 // true if sensor is healthy
        bool alt_ok:1;                  // true if calculated altitude is ok
        bool calibrated:1;              // true if calculated calibrated successfully
        float WaterDepth;                 // pressure in Pascal
    } sensors[SONNER_MAX_INSTANCES];

    AP_Float                            _alt_offset;
    float                               _alt_offset_active;
    AP_Int8                             _primary_sonner; // primary chosen by user

    // when did we last notify the GCS of new pressure reference?
    uint32_t                            _last_notify_ms;

    bool _add_backend(AP_Sonner_Backend *backend);
    AP_Int8                            _filter_range;  // valid value range from mean value
    AP_Int32                           _sonner_probe_ext;
};

namespace AP {
    AP_Sonner &sonner();
};
