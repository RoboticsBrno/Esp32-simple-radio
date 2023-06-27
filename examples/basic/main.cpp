#ifdef LX16A_ARDUINO
#include <Arduino.h>
#else
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
static void delay(int ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
#endif

void setup() {
   
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
