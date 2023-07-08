# Esp32-simple-radio

Library that provides very simple radio transmission via BLE GAP, inspired by micro:bit's radio.

## Example

Add current version to your platformio.ini:

```ini
...
lib_deps = https://github.com/RoboticsBrno/Esp32-simple-radio/archive/refs/tags/v1.0.0.zip # or newer version...
...
```

```cpp
#include "simple_radio.h"

void setup() {
    SimpleRadio.begin(0);

    SimpleRadio.setOnStringCallback([](std::string str, PacketInfo info) {
        printf("Got string %s rssi %d\n", str.c_str(), info.rssi);
    });

    SimpleRadio.setOnNumberCallback([](double val, PacketInfo info) {
        printf("Got number %f rssi %d\n", val, info.rssi);
    });

    SimpleRadio.setOnKeyValueCallback([](std::string key, double val, PacketInfo info) {
        printf("Got kv %s = %f rssi %d\n", key.c_str(), val, info.rssi);
    });

    char buf[32];
    for (size_t i = 0; true; ++i) {
        switch (i % 3) {
        case 0:
            snprintf(buf, sizeof(buf), "iter %d", i);
            SimpleRadio.sendString(buf);
            break;
        case 1:
            SimpleRadio.sendNumber(double(i) / 12);
            break;
        case 2:
            SimpleRadio.sendKeyValue("num", i);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```
