#ifdef LX16A_ARDUINO
#include <Arduino.h>
#else
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
static void delay(int ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
#endif

#include "simple_radio.h"
#include <cstring>
#include <span>

void setup() {
    SimpleRadio.begin(0);

    SimpleRadio.setOnStringCallback(
        [](std::string str, PacketInfo info) { printf("Got string %s rssi %d\n", str.c_str(), info.rssi); });

    SimpleRadio.setOnNumberCallback(
        [](double val, PacketInfo info) { printf("Got number %f rssi %d\n", val, info.rssi); });

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
        case 0: {
            snprintf(buf, sizeof(buf), "iter %d", i);
            SimpleRadio.sendString(buf);
            break;
        }
        case 1: {
            SimpleRadio.sendNumber(double(i) / 12);
            break;
        }
        case 2: {
            SimpleRadio.sendKeyValue("num", i);
            break;
        }
        case 3: {
            const uint8_t blob[] = {0xde, 0xad, 0xbe, 0xef, static_cast<uint8_t>(i)};
            SimpleRadio.sendBlob(blob, sizeof(blob));
            break;
        }
        }

        auto free = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
        printf("Free mem: %d\n", free);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void loop() {
}

#ifndef ARDUINO
extern "C" void app_main() {
    setup();
    while (true) {
        loop();
        vTaskDelay(0);
    }
}
#endif
