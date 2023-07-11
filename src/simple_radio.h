#pragma once

#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include <atomic>
#include <cstring>
#include <esp_err.h>
#include <functional>
#include <mutex>

enum PacketDataType : uint8_t {
    String = 0,
    Number = 1, // double
    KeyValue = 2, // string key, double value
};

struct PacketInfo {
    uint8_t group;
    esp_bd_addr_t addr;
    int16_t rssi; // signal strength, higher == better
};

typedef std::function<void(std::string, PacketInfo)> PacketStringCallbackT;
typedef std::function<void(double, PacketInfo)> PacketNumberCallbackT;
typedef std::function<void(std::string, double, PacketInfo)> PacketKeyValueCallbackT;

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

        // Stop advertising message after N ms, so that it won't be received by others
        // long after it was "sent".
        // Set to 0 to disable.
        // Default: 1000 ms
        uint16_t message_timeout_ms;

        /*!< Minimum advertising interval, "how often" is message transmitted
            Range: 0x0020 to 0x4000 Default: N = 0x0020 (12.5 ms)
            Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec */
        uint16_t adv_int_min;
        /*!< Maximum advertising interval "how often" is message transmitted
            Range: 0x0020 to 0x4000 Default: N = 0x0040 (25 ms)
            Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec Advertising max interval */
        uint16_t adv_int_max;

        /*!< Scan interval, "how often" to scan for new messages.
             Range: 0x0004 to 0x4000 Default: 0x0010 (10 ms)
             Time = N * 0.625 msec
             Time Range: 2.5 msec to 10.24 seconds*/
        uint16_t scan_interval;
        /*!< Scan window. The duration of the LE scan. LE_Scan_Window
             shall be less than or equal to LE_Scan_Interval
             Range: 0x0004 to 0x4000 Default: 0x0010 (10 ms)
             Time = N * 0.625 msec
             Time Range: 2.5 msec to 10240 msec */
        uint16_t scan_window;
    };

    static const Config DEFAULT_CONFIG;

    SimpleRadioImpl();
    SimpleRadioImpl(const SimpleRadioImpl&) = delete;
    ~SimpleRadioImpl();

    esp_err_t begin(uint8_t group = 0, const SimpleRadioImpl::Config& config = DEFAULT_CONFIG);
    void end();

    // group in range <0,16)
    void setGroup(uint8_t group);

    uint8_t group() const;

    // Returns local address others see this device as. Works only after begin() is called.
    esp_err_t address(esp_bd_addr_t out_address) const;

    // len must be <= 30
    void setData(PacketDataType dtype, const uint8_t* data, size_t len);

    void sendString(const char* str) {
        setData(PacketDataType::String, (const uint8_t*)str, strlen(str));
    }
    void sendString(const std::string& str) {
        setData(PacketDataType::String, (const uint8_t*)str.c_str(), str.size());
    }

    void sendNumber(double number) {
        setData(PacketDataType::Number, (const uint8_t*)&number, sizeof(double));
    }

    void sendKeyValue(const std::string& key, double value) {
        char buf[30];
        auto key_len = std::min(sizeof(buf) - 8, key.size());
        memcpy(buf, (void*)&value, 8);
        memcpy(buf + 8, (void*)key.c_str(), key_len);
        setData(PacketDataType::KeyValue, (const uint8_t*)buf, key_len + 8);
    }

    // Ignore re-transmitted messages with same content, call OnMessageCallback only when data changes.
    // Default true.
    void setIgnoreRepeatedMessages(bool ignore) {
        m_mutex.lock();
        m_ignore_repeated_messages = ignore;
        m_mutex.unlock();
    }

    void setOnStringCallback(PacketStringCallbackT cb) {
        m_mutex.lock();
        m_cb_string = cb;
        m_mutex.unlock();
    }

    void setOnNumberCallback(PacketNumberCallbackT cb) {
        m_mutex.lock();
        m_cb_number = cb;
        m_mutex.unlock();
    }

    void setOnKeyValueCallback(PacketKeyValueCallbackT cb) {
        m_mutex.lock();
        m_cb_keyvalue = cb;
        m_mutex.unlock();
    }

private:
    static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
    static void onTimeout(TimerHandle_t timer);

    void submitAdvertisingData();

    std::function<void(PacketInfo)> prepareCallbackLocked(PacketDataType dtype, const uint8_t* data, size_t len);

    std::atomic<bool> m_initialized;
    bool m_ignore_repeated_messages;
    bool m_is_advertising;

    uint8_t m_data[31];
    uint8_t m_data_size;

    uint8_t m_last_incomming[31];
    uint8_t m_last_incomming_len;

    Config m_used_config;

    PacketStringCallbackT m_cb_string;
    PacketNumberCallbackT m_cb_number;
    PacketKeyValueCallbackT m_cb_keyvalue;

    TimerHandle_t m_timeout_timer;

    mutable std::mutex m_mutex;
};

extern SimpleRadioImpl SimpleRadio;
