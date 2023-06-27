#pragma once

#include "esp_gap_ble_api.h"
#include <esp_err.h>
#include <functional>
#include <mutex>

struct MessageWrapper {
    uint8_t data[30];
    uint8_t data_len;
    uint8_t group;
    esp_bd_addr_t addr;
    int16_t rssi; // signal strength, higher == better
    bool repeated;
};

typedef std::function<void(MessageWrapper)> MessageCallbackT;

class SimpleRadioImpl {
public:
    struct Config {
        bool init_nvs;
        bool init_bt_controller;
        // Releases memory of the classic, non-BLE bluetooth stack
        // because we don't use it, to free up RAM. It cannot be reversed.
        // Only works if init_bt_controller == true.
        bool release_bt_memory;
        bool init_bluedroid;
    };

    static const Config DEFAULT_CONFIG;

    SimpleRadioImpl();
    SimpleRadioImpl(const SimpleRadioImpl&) = delete;
    ~SimpleRadioImpl();

    esp_err_t begin(uint8_t group = 0, const SimpleRadioImpl::Config& config = DEFAULT_CONFIG);
    void end();

    // group in range <0,64)
    void setGroup(uint8_t group);

    // len must be <= 30
    void setData(const uint8_t *data, size_t len);

    void setOnMessageCallback(MessageCallbackT callback) {
        m_mutex.lock();
        m_msg_callback = callback;
        m_mutex.unlock();
    }

private:
    static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

    bool m_initialized;
    uint8_t m_data[31];
    size_t m_data_size;
    Config m_used_config;
    MessageCallbackT m_msg_callback;
    mutable std::mutex m_mutex;
};

extern SimpleRadioImpl SimpleRadio;
