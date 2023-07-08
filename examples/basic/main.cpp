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

void setup() {
    SimpleRadio.begin(0);

    SimpleRadio.setOnStringCallback(
        [](std::string str, PacketInfo info) { printf("Got string %s rssi %d\n", str.c_str(), info.rssi); });

    SimpleRadio.setOnNumberCallback(
        [](double val, PacketInfo info) { printf("Got number %f rssi %d\n", val, info.rssi); });

    SimpleRadio.setOnKeyValueCallback([](std::string key, double val, PacketInfo info) {
        printf("Got kv %s = %f rssi %d\n", key.c_str(), val, info.rssi);
    });

    char buf[32];
    for (size_t i = 0; true; ++i) {
        switch (i % 3) {
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
