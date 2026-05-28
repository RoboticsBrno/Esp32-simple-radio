#include "simple_radio.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include <algorithm>
#include <cstring>

#define TAG "SimpleRadio"

namespace {

const uint8_t kBroadcastAddress[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
constexpr uint8_t kMagic0 = 'S';
constexpr uint8_t kMagic1 = 'R';
constexpr size_t kHeaderSize = 6;
constexpr size_t kMaxPayloadSize = 1490;

struct DecodedPacket {
    uint8_t group = 0;
    PacketDataType type = PacketDataType::String;
    const uint8_t* payload = nullptr;
    size_t payload_len = 0;
};

size_t encodePacket(PacketDataType type, uint8_t group, const uint8_t* payload, size_t payload_len, uint8_t* out, size_t out_size) {
    if (out == nullptr || out_size < kHeaderSize ||
        payload_len > kMaxPayloadSize ||
        kHeaderSize + payload_len > out_size) {
        return 0;
    }
    if (payload_len > 0 && payload == nullptr) {
        return 0;
    }

    out[0] = kMagic0;
    out[1] = kMagic1;
    out[2] = group;
    out[3] = static_cast<uint8_t>(type);
    out[4] = static_cast<uint8_t>(payload_len & 0xFF);
    out[5] = static_cast<uint8_t>((payload_len >> 8) & 0xFF);

    if (payload_len > 0) {
        std::memcpy(out + kHeaderSize, payload, payload_len);
    }

    return kHeaderSize + payload_len;
}

bool decodePacket(const uint8_t* packet, size_t packet_len, DecodedPacket& out) {
    if (packet == nullptr || packet_len < kHeaderSize) {
        return false;
    }
    if (packet[0] != kMagic0 || packet[1] != kMagic1) {
        return false;
    }

    const size_t payload_len = static_cast<size_t>(packet[4]) | (static_cast<size_t>(packet[5]) << 8);
    if (packet_len != kHeaderSize + payload_len) {
        return false;
    }

    const auto type = static_cast<PacketDataType>(packet[3]);
    if (type > PacketDataType::Blob) {
        return false;
    }

    out.group = packet[2];
    out.type = type;
    out.payload = packet + kHeaderSize;
    out.payload_len = payload_len;
    return true;
}

PacketInfo makePacketInfo(const uint8_t* addr, uint8_t group, int16_t rssi) {
    PacketInfo info = {};
    info.group = group;
    if (addr != nullptr) {
        std::memcpy(info.addr, addr, sizeof(info.addr));
    }
    info.rssi = rssi;
    return info;
}

} // namespace

SimpleRadioImpl SimpleRadio;

const SimpleRadioImpl::Config SimpleRadioImpl::DEFAULT_CONFIG = {
    .init_nvs = true,
    .init_netif = true,
    .init_event_loop = true,
    .init_wifi = true,
    .init_esp_now = true,
    .channel = 1,
};

SimpleRadioImpl::SimpleRadioImpl()
    : m_initialized(false),
      m_ignore_repeated_messages(true),
      m_group(0),
      m_tx_buffer(kMaxPacketSize, 0),
      m_last_incoming_addr({0, 0, 0, 0, 0, 0}) {
}

SimpleRadioImpl::~SimpleRadioImpl() {
}

esp_err_t SimpleRadioImpl::initNvsIfNeeded(const Config& config) {
    if (!config.init_nvs) {
        return ESP_OK;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }
    return ret;
}

esp_err_t SimpleRadioImpl::initWifiIfNeeded(const Config& config) {
    if (config.init_netif) {
        esp_err_t ret = esp_netif_init();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            return ret;
        }
    }

    if (config.init_event_loop) {
        esp_err_t ret = esp_event_loop_create_default();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            return ret;
        }
    }

    if (!config.init_wifi) {
        return ESP_OK;
    }

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&wifi_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_CONN) {
        return ret;
    }

    ret = esp_wifi_set_channel(config.channel, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t SimpleRadioImpl::initEspNowIfNeeded(const Config& config) {
    if (!config.init_esp_now) {
        return ESP_OK;
    }

    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_now_register_recv_cb(onDataReceived);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_now_register_send_cb(onDataSent);
    if (ret != ESP_OK) {
        return ret;
    }

    esp_now_peer_info_t peer = {};
    std::memcpy(peer.peer_addr, kBroadcastAddress, sizeof(peer.peer_addr));
    peer.ifidx = WIFI_IF_STA;
    peer.channel = config.channel;
    peer.encrypt = false;

    ret = esp_now_add_peer(&peer);
    if (ret == ESP_ERR_ESPNOW_EXIST) {
        return ESP_OK;
    }
    return ret;
}

esp_err_t SimpleRadioImpl::begin(uint8_t group, const SimpleRadioImpl::Config& config) {
    if (m_initialized) {
        ESP_LOGE(TAG, "SimpleRadio is already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    m_used_config = config;
    m_group = group;

    esp_err_t ret = initNvsIfNeeded(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s failed to init nvs flash: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = initWifiIfNeeded(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s failed to init Wi-Fi: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = initEspNowIfNeeded(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s failed to init ESP-NOW: %s", __func__, esp_err_to_name(ret));
        if (config.init_wifi) {
            esp_wifi_stop();
            esp_wifi_deinit();
        }
        return ret;
    }

    uint32_t version = 0;
    ret = esp_now_get_version(&version);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ESP-NOW version %lu", static_cast<unsigned long>(version));
    }

    m_initialized = true;
    return ESP_OK;
}

void SimpleRadioImpl::end() {
    if (!m_initialized) {
        ESP_LOGE(TAG, "SimpleRadio is not initialized");
        return;
    }

    if (m_used_config.init_esp_now) {
        esp_now_del_peer(kBroadcastAddress);
        esp_now_unregister_recv_cb();
        esp_now_unregister_send_cb();
        esp_now_deinit();
    }

    if (m_used_config.init_wifi) {
        esp_wifi_stop();
        esp_wifi_deinit();
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_initialized = false;
    m_group = 0;
    m_cb_string = nullptr;
    m_cb_number = nullptr;
    m_cb_keyvalue = nullptr;
    m_cb_blob = nullptr;
    m_last_incoming_packet.clear();
    std::fill(m_last_incoming_addr.begin(), m_last_incoming_addr.end(), 0);
}

void SimpleRadioImpl::setGroup(uint8_t group) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_group = group;
}

uint8_t SimpleRadioImpl::group() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_group;
}

esp_err_t SimpleRadioImpl::address(simple_radio_addr_t out_address) const {
    return esp_wifi_get_mac(WIFI_IF_STA, out_address);
}

void SimpleRadioImpl::setData(PacketDataType dtype, const uint8_t* data, size_t len) {
    if (!m_initialized) {
        ESP_LOGE(TAG, "SimpleRadio is not initialized");
        return;
    }

    esp_err_t ret = ESP_OK;
    size_t packet_len = 0;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        packet_len = encodePacket(dtype, m_group, data, len, m_tx_buffer.data(), m_tx_buffer.size());
        if (packet_len != 0) {
            ret = esp_now_send(kBroadcastAddress, m_tx_buffer.data(), packet_len);
        }
    }

    if (packet_len == 0) {
        ESP_LOGE(TAG, "failed to encode outgoing packet type=%u len=%u", static_cast<unsigned>(dtype), static_cast<unsigned>(len));
        return;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(ret));
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
void SimpleRadioImpl::onDataSent(const esp_now_send_info_t* tx_info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        return;
    }

    if (tx_info == nullptr || tx_info->des_addr == nullptr) {
        ESP_LOGW(TAG, "ESP-NOW send failed");
        return;
    }

    ESP_LOGW(TAG, "ESP-NOW send failed to %02x:%02x:%02x:%02x:%02x:%02x",
        tx_info->des_addr[0], tx_info->des_addr[1], tx_info->des_addr[2],
        tx_info->des_addr[3], tx_info->des_addr[4], tx_info->des_addr[5]);
}
#else
void SimpleRadioImpl::onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        return;
    }

    if (mac_addr == nullptr) {
        ESP_LOGW(TAG, "ESP-NOW send failed");
        return;
    }

    ESP_LOGW(TAG, "ESP-NOW send failed to %02x:%02x:%02x:%02x:%02x:%02x",
        mac_addr[0], mac_addr[1], mac_addr[2],
        mac_addr[3], mac_addr[4], mac_addr[5]);
}
#endif

SimpleRadioImpl::PendingCallback SimpleRadioImpl::prepareCallbackLocked(PacketDataType dtype, const uint8_t* data, size_t len) {
    using namespace std::placeholders;

    PendingCallback pending = {};

    switch (dtype) {
    case PacketDataType::String: {
        if (!m_cb_string) {
            return pending;
        }

        std::string str(reinterpret_cast<const char*>(data), len);
        pending.callback = std::bind(m_cb_string, str, _1);
        pending.valid = true;
        return pending;
    }
    case PacketDataType::Number: {
        if (!m_cb_number) {
            return pending;
        }

        if (len != sizeof(double)) {
            ESP_LOGE(TAG, "invalid number packet received, got len %u instead of %u",
                static_cast<unsigned>(len), static_cast<unsigned>(sizeof(double)));
            return pending;
        }

        double value = 0;
        std::memcpy(&value, data, sizeof(value));
        pending.callback = std::bind(m_cb_number, value, _1);
        pending.valid = true;
        return pending;
    }
    case PacketDataType::KeyValue: {
        if (!m_cb_keyvalue) {
            return pending;
        }

        if (len < sizeof(double)) {
            ESP_LOGE(TAG, "invalid keyvalue packet received, got len %u instead of >= %u",
                static_cast<unsigned>(len), static_cast<unsigned>(sizeof(double)));
            return pending;
        }

        double value = 0;
        std::memcpy(&value, data, sizeof(value));
        std::string key(reinterpret_cast<const char*>(data + sizeof(double)), len - sizeof(double));
        pending.callback = std::bind(m_cb_keyvalue, key, value, _1);
        pending.valid = true;
        return pending;
    }
    case PacketDataType::Blob: {
        if (!m_cb_blob) {
            return pending;
        }

        pending.callback = std::bind(m_cb_blob, std::span<const uint8_t>(data, len), _1);
        pending.valid = true;
        return pending;
    }
    default:
        ESP_LOGE(TAG, "invalid data type received: %u", static_cast<unsigned>(dtype));
        return pending;
    }
}

void SimpleRadioImpl::onDataReceived(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int len) {
    if (esp_now_info == nullptr || esp_now_info->src_addr == nullptr || data == nullptr || len <= 0) {
        return;
    }

    auto& self = SimpleRadio;
    DecodedPacket decoded = {};
    if (!decodePacket(data, static_cast<size_t>(len), decoded)) {
        return;
    }

    int16_t rssi = 0;
    if (esp_now_info->rx_ctrl != nullptr) {
        rssi = esp_now_info->rx_ctrl->rssi;
    }

    PendingCallback pending = {};
    {
        std::lock_guard<std::mutex> lock(self.m_mutex);
        if (decoded.group != self.m_group) {
            return;
        }

        if (self.m_ignore_repeated_messages &&
            self.m_last_incoming_packet.size() == static_cast<size_t>(len) &&
            std::memcmp(self.m_last_incoming_addr.data(), esp_now_info->src_addr, self.m_last_incoming_addr.size()) == 0 &&
            std::memcmp(self.m_last_incoming_packet.data(), data, static_cast<size_t>(len)) == 0) {
            return;
        }

        self.m_last_incoming_packet.assign(data, data + len);
        std::memcpy(self.m_last_incoming_addr.data(), esp_now_info->src_addr, self.m_last_incoming_addr.size());
        pending = self.prepareCallbackLocked(decoded.type, decoded.payload, decoded.payload_len);
    }

    if (!pending.valid) {
        return;
    }

    pending.callback(makePacketInfo(esp_now_info->src_addr, decoded.group, rssi));
}
