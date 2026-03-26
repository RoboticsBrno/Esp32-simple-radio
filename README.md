# Esp32-simple-radio

Library that provides very simple radio transmission via ESP-NOW broadcast, inspired by micro:bit's radio.

This rewrite no longer uses BLE advertising. It now uses ESP-NOW v2 frames, which raises the practical payload limit from 30 bytes to up to 1490 bytes per message.

## Behavior

- Broadcast-only transport
- 8-bit group IDs (`0-255`)
- Full 6-byte sender MAC in `PacketInfo`
- Single-frame messages only
- Typed helpers for string, number, key/value, plus raw blob messages

`setData()` is the low-level send primitive used by the helper methods. It now encodes and transmits one ESP-NOW packet immediately.

## Example

Add current version to your platformio.ini:

```ini
...
lib_deps = https://github.com/RoboticsBrno/Esp32-simple-radio/archive/refs/tags/v1.0.0.zip # or newer version...
...
```

```cpp
#include "simple_radio.h"
#include <span>

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

    SimpleRadio.setOnBlobCallback([](std::span<const uint8_t> blob, PacketInfo info) {
        printf("Got blob len=%u rssi %d from %02x:%02x:%02x:%02x:%02x:%02x\n",
            static_cast<unsigned>(blob.size()),
            info.rssi,
            info.addr[0], info.addr[1], info.addr[2], info.addr[3], info.addr[4], info.addr[5]);
    });

    char buf[32];
    for (size_t i = 0; true; ++i) {
        switch (i % 4) {
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
        case 3: {
            const uint8_t blob[] = {0xde, 0xad, 0xbe, 0xef, static_cast<uint8_t>(i)};
            SimpleRadio.sendBlob(blob, sizeof(blob));
            break;
        }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

## Notes

- All participating devices need to use the same Wi-Fi channel.
- This library uses broadcast ESP-NOW peers only.
- ESP-NOW v2 extended frames are required for payloads above 250 bytes.
- Blob receive callbacks use `std::span`, so consumers need C++20 or newer.
