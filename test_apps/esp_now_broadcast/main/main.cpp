#include "simple_radio.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdio>
#include <cstring>
#include <span>

namespace {

constexpr char TAG[] = "radio_test";

void print_mac(const char* prefix, const uint8_t* addr) {
    printf("%s%02x:%02x:%02x:%02x:%02x:%02x\n",
        prefix, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

} // namespace

extern "C" void app_main(void) {
    const esp_err_t begin_result = SimpleRadio.begin(7);
    if (begin_result != ESP_OK) {
        ESP_LOGE(TAG, "SimpleRadio.begin failed: %s", esp_err_to_name(begin_result));
        return;
    }

    simple_radio_addr_t local_addr = {0};
    if (SimpleRadio.address(local_addr) == ESP_OK) {
        print_mac("LOCAL ", local_addr);
    }

    SimpleRadio.setOnStringCallback([](std::string str, PacketInfo info) {
        printf("RX string from %02x:%02x:%02x:%02x:%02x:%02x rssi=%d payload=%s\n",
            info.addr[0], info.addr[1], info.addr[2], info.addr[3], info.addr[4], info.addr[5], info.rssi, str.c_str());
    });

    SimpleRadio.setOnBlobCallback([](std::span<const uint8_t> blob, PacketInfo info) {
        printf("RX blob from %02x:%02x:%02x:%02x:%02x:%02x rssi=%d len=%u last=%u\n",
            info.addr[0], info.addr[1], info.addr[2], info.addr[3], info.addr[4], info.addr[5], info.rssi,
            static_cast<unsigned>(blob.size()),
            blob.empty() ? 0u : static_cast<unsigned>(blob.back()));
    });

    char message[96];
    std::snprintf(message, sizeof(message), "hello from %02x%02x", local_addr[4], local_addr[5]);

    uint8_t counter = 0;
    while (true) {
        SimpleRadio.sendString(message);

        uint8_t blob[32] = {};
        for (size_t i = 0; i < sizeof(blob); ++i) {
            blob[i] = static_cast<uint8_t>(counter + i);
        }
        SimpleRadio.sendBlob(blob, sizeof(blob));

        printf("TX round counter=%u\n", static_cast<unsigned>(counter));
        ++counter;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
