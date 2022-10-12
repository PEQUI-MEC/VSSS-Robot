#ifndef BatteryWatcher_H
#define BatteryWatcher_H

#include <array>
#include "mbed.h"
#include "PIN_MAP.h"

class BatteryWatcher {
    AnalogIn battery_vin;
    std::array<DigitalOut, 4> LEDs;
    
    void led_write(uint8_t num);

    public:

    BatteryWatcher();
    void update_battery_leds();
};

#endif //BatteryWatcher_H