#include <cstring>

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs_flash.h"

#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include "simple_radio.h"

#define TAG "SimpleRadio"

// esp_ble_gap_register_callback does not allow to pass a user argument,
// and we need to be able to access the SimpleRadioImpl instance from
// the event callback. We need to have a SimpleRadio singleton, arduino-style for this.
SimpleRadioImpl SimpleRadio;

static constexpr const uint8_t SIMPLERADIO_BLE_ADV_PROP_TYPE = 0x80;

const SimpleRadioImpl::Config SimpleRadioImpl::DEFAULT_CONFIG = {
    .init_nvs = true,
    .init_bt_controller = true,
    .release_bt_memory = true,
    .init_bluedroid = true,
    .message_timeout_ms = 1000,
    .adv_int_min = 0x0020,
    .adv_int_max = 0x0040,
    .scan_interval = 0,
    .scan_window = 0,
};

SimpleRadioImpl::SimpleRadioImpl() {
    m_ignore_repeated_messages = true;
    m_initialized = false;
    m_is_advertising = false;
    m_last_incomming_len = 0;
    m_data_size = 0;
    m_data[0] = SIMPLERADIO_BLE_ADV_PROP_TYPE;
    m_timeout_timer = nullptr;
}

SimpleRadioImpl::~SimpleRadioImpl() {
}

esp_err_t SimpleRadioImpl::begin(uint8_t group, const SimpleRadioImpl::Config& config) {
    if (m_initialized) {
        ESP_LOGE(TAG, "SimpleRadio is already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (group >= 16) {
        ESP_LOGE(TAG, "The group id must be in range <0;16)");
        return ESP_ERR_INVALID_ARG;
    }

    m_used_config = config;

    esp_err_t ret = ESP_OK;
    esp_ble_scan_params_t scan_params = {};

    if (config.init_nvs) {
        ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_erase();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "%s failed to erase nvs flash: %s", __func__, esp_err_to_name(ret));
                return ret;
            }
            ret = nvs_flash_init();
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "%s failed to init nvs flash: %s", __func__, esp_err_to_name(ret));
            return ret;
        }
    }

    if (config.init_bt_controller) {
        if (config.release_bt_memory) {
            // Releases memory of the classic, non-BLE bluetooth stack
            // because we don't use it, to free up RAM. It cannot be reversed.
            ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "%s failed to release bt controller memory: %s", __func__, esp_err_to_name(ret));
                return ret;
            }
        }

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
#if !defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_IDF_TARGET_ESP32C3)
        bt_cfg.mode = ESP_BT_MODE_BLE;
#else
        bt_cfg.bluetooth_mode = ESP_BT_MODE_BLE;
#endif
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
            return ret;
        }

        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret) {
            ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
            goto exit_bt_deinit;
        }
    }

    if (config.init_bluedroid) {
        ret = esp_bluedroid_init();
        if (ret) {
            ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
            goto exit_bt_disable;
        }
        ret = esp_bluedroid_enable();
        if (ret) {
            ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
            goto exit_bluedroid_deinit;
        }
    }

    ret = esp_ble_gap_register_callback(gapEventHandler);
    if (ret) {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        goto exit_bluedroid_disable;
    }

    scan_params.scan_type = BLE_SCAN_TYPE_PASSIVE;
    scan_params.scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE;
    scan_params.scan_window = config.scan_window;
    scan_params.scan_interval = config.scan_interval;

    ret = esp_ble_gap_set_scan_params(&scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gap set scan params error, error code = %x", ret);
        goto exit_bluedroid_disable;
    }

    setGroup(group);
    m_initialized = true;

    if (config.message_timeout_ms) {
        m_timeout_timer = xTimerCreate("sradio_timeout", pdMS_TO_TICKS(config.message_timeout_ms), pdFALSE, nullptr, onTimeout);
    }

    submitAdvertisingData();

    return ESP_OK;

exit_bluedroid_disable:
    if (config.init_bluedroid)
        esp_bluedroid_disable();
exit_bluedroid_deinit:
    if (config.init_bluedroid)
        esp_bluedroid_deinit();
exit_bt_disable:
    if (config.init_bt_controller)
        esp_bt_controller_disable();
exit_bt_deinit:
    if (config.init_bt_controller)
        esp_bt_controller_deinit();
    return ret;
}

void SimpleRadioImpl::end() {
    if (!m_initialized) {
        ESP_LOGE(TAG, "SimpleRadio is not initialized");
        return;
    }

    esp_err_t ret = esp_ble_gap_stop_scanning();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_ble_gap_stop_scanning error, error code = %x", ret);
    }

    ret = esp_ble_gap_stop_advertising();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_ble_gap_stop_advertising error, error code = %x", ret);
    }

    if (m_used_config.init_bluedroid) {
        ret = esp_bluedroid_disable();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_bluedroid_disable error, error code = %x", ret);
        }

        ret = esp_bluedroid_deinit();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_bluedroid_deinit error, error code = %x", ret);
        }
    }

    if (m_used_config.init_bt_controller) {
        ret = esp_bt_controller_disable();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_bt_controller_disable error, error code = %x", ret);
        }

        ret = esp_bt_controller_deinit();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_bt_controller_deinit error, error code = %x", ret);
        }
    }

    m_mutex.lock();
    m_is_advertising = false;
    m_initialized = false;
    m_cb_string = nullptr;
    m_cb_number = nullptr;
    m_cb_keyvalue = nullptr;
    m_last_incomming_len = 0;
    m_data_size = 0;

    if (m_timeout_timer) {
        xTimerDelete(m_timeout_timer, 0);
        m_timeout_timer = nullptr;
    }

    m_mutex.unlock();
}

void SimpleRadioImpl::gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    auto& self = SimpleRadio;

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: {
        std::lock_guard<std::mutex> l(self.m_mutex);

        if (self.m_timeout_timer) {
            xTimerReset(self.m_timeout_timer, 0);
        }

        if (self.m_is_advertising) {
            break;
        }

        static esp_ble_adv_params_t adv_params = {
            .adv_int_min = self.m_used_config.adv_int_min,
            .adv_int_max = self.m_used_config.adv_int_max,
            .adv_type = ADV_TYPE_NONCONN_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .peer_addr = {},
            .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };

        auto ret = esp_ble_gap_start_advertising(&adv_params);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "gap esp_ble_gap_start_advertising error, error code = %x", ret);
            break;
        }

        self.m_is_advertising = true;
        break;
    }
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        auto ret = esp_ble_gap_start_scanning(0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "gap start scanning error, error code = %x", ret);
        }
        break;
    }
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising stop failed");
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        const auto& d = param->scan_rst;
        if (d.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
            break;
        }

        if (d.adv_data_len == 0) {
            break;
        }

        // check if data stars with our type in top 2 bits
        if ((d.ble_adv[0] & 0xC0) != SIMPLERADIO_BLE_ADV_PROP_TYPE) {
            break;
        }

        // check if in same group
        if ((d.ble_adv[0] & 0x0F) != (self.m_data[0] & 0x0F)) {
            break;
        }

        const uint8_t* data = d.ble_adv + 1;
        const size_t data_len = d.adv_data_len - 1;

        const auto data_type = (PacketDataType)((d.ble_adv[0] >> 4) & 0x03);

        self.m_mutex.lock();
        if (self.m_ignore_repeated_messages) {
            if (self.m_last_incomming_len == d.adv_data_len && memcmp(d.ble_adv, self.m_last_incomming, d.adv_data_len) == 0) {
                self.m_mutex.unlock();
                break;
            }
            memcpy(self.m_last_incomming, d.ble_adv, d.adv_data_len);
            self.m_last_incomming_len = d.adv_data_len;
        }

        auto callback = self.prepareCallbackLocked(data_type, data, data_len);
        self.m_mutex.unlock();

        if (callback) {
            PacketInfo info;
            info.group = d.ble_adv[0] & 0x0F;
            memcpy(info.addr, d.bda, sizeof(info.addr));
            info.rssi = d.rssi;
            callback(info);
        }
        break;
    }
    default:
        break;
    }
}

void SimpleRadioImpl::onTimeout(TimerHandle_t timer) {
    auto& self = SimpleRadio;

    self.m_mutex.lock();
    if (self.m_is_advertising) {
        self.m_is_advertising = false;
        auto err = esp_ble_gap_stop_advertising();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to stop advertising due tu timeout: %x", err);
        }
    }
    self.m_mutex.unlock();
}

std::function<void(PacketInfo)> SimpleRadioImpl::prepareCallbackLocked(PacketDataType dtype, const uint8_t* data, size_t len) {
    using namespace std::placeholders;

    switch (dtype) {
    case PacketDataType::String: {
        if (!m_cb_string) {
            return std::function<void(PacketInfo)>();
        }
        std::string str((const char*)data, len);
        return std::bind(m_cb_string, str, _1);
    }
    case PacketDataType::Number: {
        if (!m_cb_number) {
            return std::function<void(PacketInfo)>();
        }

        if (len != 8) {
            ESP_LOGE(TAG, "invalid number packet received, got len %d instead of 8", len);
            return std::function<void(PacketInfo)>();
        }

        double val = *((double*)data);
        return std::bind(m_cb_number, val, _1);
    }
    case PacketDataType::KeyValue: {
        if (!m_cb_keyvalue) {
            return std::function<void(PacketInfo)>();
        }

        if (len < 8) {
            ESP_LOGE(TAG, "invalid keyvalue packet received, got len %d instead >= 8", len);
            return std::function<void(PacketInfo)>();
        }

        double val = *((double*)data);
        std::string key((const char*)data + 8, len - 8);

        return std::bind(m_cb_keyvalue, key, val, _1);
    }
    default:
        ESP_LOGE(TAG, "invalid data type received: %d", dtype);
        return std::function<void(PacketInfo)>();
    }
}

void SimpleRadioImpl::setGroup(uint8_t group) {
    if (group >= 16) {
        ESP_LOGE(TAG, "The group id must be in range <0;16)");
        return;
    }

    // check if current group (bottom 4 bits of m_data[0]) is same as set one
    if (group == (m_data[0] & 0x0F)) {
        return;
    }

    m_data[0] = (m_data[0] & 0xF0) | group;
    submitAdvertisingData();
}

uint8_t SimpleRadioImpl::group() const {
    return m_data[0] & 0x0F;
}

esp_err_t SimpleRadioImpl::address(esp_bd_addr_t out_address) const {
    uint8_t addr_type;
    return esp_ble_gap_get_local_used_addr(out_address, &addr_type);
}

void SimpleRadioImpl::setData(PacketDataType dtype, const uint8_t* data, size_t len) {
    if (len == 0) {
        m_data_size = 0;
        return;
    }

    if (len > 30) {
        ESP_LOGW(TAG, "The simple radio data can be only 30 bytes long.");
        len = 30;
    }

    m_data[0] = (m_data[0] & 0xCF) | ((dtype & 0x3) << 4);

    memcpy(m_data + 1, data, len);
    m_data_size = len + 1;

    submitAdvertisingData();
}

void SimpleRadioImpl::submitAdvertisingData() {
    if (!m_initialized || m_data_size == 0) {
        return;
    }

    m_mutex.lock();
    esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(m_data, m_data_size);
    if (raw_adv_ret) {
        ESP_LOGE(TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
    }
    m_mutex.unlock();
}
