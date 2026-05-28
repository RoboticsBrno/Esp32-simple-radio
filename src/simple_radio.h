#pragma once

#include "freertos/FreeRTOS.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <esp_err.h>
#include <esp_idf_version.h>
#include <esp_now.h>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <vector>

typedef uint8_t simple_radio_addr_t[6];

struct PacketInfo {
    uint8_t group;
    simple_radio_addr_t addr;
    int16_t rssi;
};

typedef std::function<void(std::string, PacketInfo)> PacketStringCallbackT;
typedef std::function<void(double, PacketInfo)> PacketNumberCallbackT;
typedef std::function<void(std::string, double, PacketInfo)> PacketKeyValueCallbackT;
typedef std::function<void(std::span<const uint8_t>, PacketInfo)> PacketBlobCallbackT;

enum PacketDataType : uint8_t {
    String = 0,
    Number = 1,
    KeyValue = 2,
    Blob = 3,
};

class SimpleRadioImpl {
public:
    struct Config {
        bool init_nvs;
        bool init_netif;
        bool init_event_loop;
        bool init_wifi;
        bool init_esp_now;
        uint8_t channel;
    };

    static const Config DEFAULT_CONFIG;

    SimpleRadioImpl();
    SimpleRadioImpl(const SimpleRadioImpl&) = delete;
    ~SimpleRadioImpl();

    esp_err_t begin(uint8_t group = 0, const SimpleRadioImpl::Config& config = DEFAULT_CONFIG);
    void end();

    void setGroup(uint8_t group);
    uint8_t group() const;

    esp_err_t address(simple_radio_addr_t out_address) const;

    void setData(PacketDataType dtype, const uint8_t* data, size_t len);

    void sendString(const char* str) {
        setData(PacketDataType::String, reinterpret_cast<const uint8_t*>(str), strlen(str));
    }
    void sendString(const std::string& str) {
        setData(PacketDataType::String, reinterpret_cast<const uint8_t*>(str.c_str()), str.size());
    }

    void sendNumber(double number) {
        setData(PacketDataType::Number, reinterpret_cast<const uint8_t*>(&number), sizeof(double));
    }

    void sendKeyValue(const std::string& key, double value) {
        std::vector<uint8_t> buf(sizeof(double) + key.size());
        memcpy(buf.data(), &value, sizeof(double));
        memcpy(buf.data() + sizeof(double), key.data(), key.size());
        setData(PacketDataType::KeyValue, buf.data(), buf.size());
    }

    void sendBlob(const uint8_t* data, size_t len) {
        setData(PacketDataType::Blob, data, len);
    }
    void sendBlob(std::span<const uint8_t> data) {
        setData(PacketDataType::Blob, data.data(), data.size());
    }
    void sendBlob(const std::vector<uint8_t>& data) {
        setData(PacketDataType::Blob, data.data(), data.size());
    }

    void setIgnoreRepeatedMessages(bool ignore) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_ignore_repeated_messages = ignore;
    }

    void setOnStringCallback(PacketStringCallbackT cb) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cb_string = cb;
    }

    void setOnNumberCallback(PacketNumberCallbackT cb) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cb_number = cb;
    }

    void setOnKeyValueCallback(PacketKeyValueCallbackT cb) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cb_keyvalue = cb;
    }

    void setOnBlobCallback(PacketBlobCallbackT cb) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cb_blob = cb;
    }

private:
    struct PendingCallback {
        std::function<void(PacketInfo)> callback;
        bool valid = false;
    };

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
    static void onDataSent(const esp_now_send_info_t* tx_info, esp_now_send_status_t status);
#else
    static void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
#endif
    static void onDataReceived(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int len);

    PendingCallback prepareCallbackLocked(PacketDataType dtype, const uint8_t* data, size_t len);
    esp_err_t initNvsIfNeeded(const Config& config);
    esp_err_t initWifiIfNeeded(const Config& config);
    esp_err_t initEspNowIfNeeded(const Config& config);

    static constexpr size_t kHeaderSize = 6;
    static constexpr size_t kMaxPayloadSize = 1490;
    static constexpr size_t kMaxPacketSize = kHeaderSize + kMaxPayloadSize;

    std::atomic<bool> m_initialized;
    bool m_ignore_repeated_messages;
    uint8_t m_group;

    Config m_used_config;

    std::vector<uint8_t> m_tx_buffer;
    std::vector<uint8_t> m_last_incoming_packet;
    std::array<uint8_t, 6> m_last_incoming_addr;

    PacketStringCallbackT m_cb_string;
    PacketNumberCallbackT m_cb_number;
    PacketKeyValueCallbackT m_cb_keyvalue;
    PacketBlobCallbackT m_cb_blob;

    mutable std::mutex m_mutex;
};

extern SimpleRadioImpl SimpleRadio;
