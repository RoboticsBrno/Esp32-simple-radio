#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

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
};
 
SimpleRadioImpl::SimpleRadioImpl() : m_data_size(0) {
    m_data[0] = SIMPLERADIO_BLE_ADV_PROP_TYPE;
}

SimpleRadioImpl::~SimpleRadioImpl() {

}

esp_err_t SimpleRadioImpl::begin(uint8_t group, const SimpleRadioImpl::Config& config) {
    if(m_initialized) {
        ESP_LOGE(TAG, "SimpleRadio is already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if(group >= 64) {
        ESP_LOGE(TAG, "The group id must be in range <0;64)");
        return ESP_ERR_INVALID_ARG;
    }

    m_used_config = config;

    esp_err_t ret = ESP_OK;
    esp_ble_scan_params_t scan_params = {};

    if(config.init_nvs) {
        ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_erase();
            if(ret != ESP_OK) {
                ESP_LOGE(TAG, "%s failed to erase nvs flash: %s", __func__, esp_err_to_name(ret));
                return ret;
            }
            ret = nvs_flash_init();
        }
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "%s failed to init nvs flash: %s", __func__, esp_err_to_name(ret));
            return ret;
        }
    }

    if(config.init_bt_controller) {
        if(config.release_bt_memory) {
            // Releases memory of the classic, non-BLE bluetooth stack
            // because we don't use it, to free up RAM. It cannot be reversed.
            ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
            if(ret != ESP_OK) {
                ESP_LOGE(TAG, "%s failed to release bt controller memory: %s", __func__, esp_err_to_name(ret));
                return ret;
            }
        }

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        bt_cfg.mode = ESP_BT_MODE_BLE;
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

    if(config.init_bluedroid) {
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
    if (ret){
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        goto exit_bluedroid_disable;
    }

    scan_params.scan_type = BLE_SCAN_TYPE_PASSIVE;
    scan_params.scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE;

    ret = esp_ble_gap_set_scan_params(&scan_params);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "gap set scan params error, error code = %x", ret);
        goto exit_bluedroid_disable;
    }

    ret = esp_ble_gap_start_scanning(0);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "gap start scanning error, error code = %x", ret);
        goto exit_bluedroid_disable;
    }

    setGroup(group);
    m_initialized = true;

    if(m_data_size > 0) {
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(m_data, m_data_size);
        if (raw_adv_ret){
            ESP_LOGE(TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
    }

    return ESP_OK;

exit_bluedroid_disable:
    if(config.init_bluedroid)
        esp_bluedroid_disable();
exit_bluedroid_deinit:
    if(config.init_bluedroid)
        esp_bluedroid_deinit();
exit_bt_disable:
    if(config.init_bt_controller)
        esp_bt_controller_disable();
exit_bt_deinit:
    if(config.init_bt_controller)
        esp_bt_controller_deinit();
    return ret;
}

void SimpleRadioImpl::end() {
    if(!m_initialized) {
        ESP_LOGE(TAG, "SimpleRadio is not initialized");
        return;
    }

    esp_err_t ret = esp_ble_gap_stop_scanning();
    if(ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_ble_gap_stop_scanning error, error code = %x", ret);
    }

    ret = esp_ble_gap_stop_advertising();
    if(ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_ble_gap_stop_advertising error, error code = %x", ret);
    }

    if(m_used_config.init_bluedroid) {
        ret = esp_bluedroid_disable();
        if(ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_bluedroid_disable error, error code = %x", ret);
        }

        ret = esp_bluedroid_deinit();
        if(ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_bluedroid_deinit error, error code = %x", ret);
        }
    }

    if(m_used_config.init_bt_controller) {
        ret = esp_bt_controller_disable();
        if(ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_bt_controller_disable error, error code = %x", ret);
        }

        ret = esp_bt_controller_deinit();
        if(ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_bt_controller_deinit error, error code = %x", ret);
        }
    }
}

void SimpleRadioImpl::gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    auto &self = SimpleRadio;

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: {
        static esp_ble_adv_params_t adv_params = {
            .adv_int_min        = 0x0400,
            .adv_int_max        = 0x0800,
            .adv_type           = ADV_TYPE_NONCONN_IND,
            .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
            .peer_addr = {},
            .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map        = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };
        esp_ble_gap_start_advertising(&adv_params);
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
        if(d.adv_data_len == 0) {
            break;
        }

        // check if data stars with our type in top 2 bits
        if((d.ble_adv[0] & 0xC0) != SIMPLERADIO_BLE_ADV_PROP_TYPE) {
            break;
        }

        // check if in same group
        if((d.ble_adv[0] & 0x3f) != (self.m_data[0] & 0x3f)) {
            break;
        }

        const uint8_t *data = d.ble_adv+1;
        const size_t data_len = d.adv_data_len-1;

        self.m_mutex.lock();
        auto callback = self.m_msg_callback;
        self.m_mutex.unlock();
        if(callback) {
            MessageWrapper msg;
            memcpy(msg.data, data, data_len);
            msg.data_len = data_len;
            msg.group = d.ble_adv[0] & 0x3f;
            memcpy(msg.addr, d.bda, sizeof(msg.addr));
            msg.rssi = d.rssi;
            callback(msg);
        }
        break;
    }
    default:
        break;
    }
}

void SimpleRadioImpl::setGroup(uint8_t group) {
    if(group >= 64) {
        ESP_LOGE(TAG, "The group id must be in range <0;64)");
        return;
    }

    // check if current group (bottom 6 bits of m_data[0]) is same as set one
    if(group == (m_data[0] & 0x3F)) {
        return;
    }

    m_data[0] = SIMPLERADIO_BLE_ADV_PROP_TYPE | group;

    if(m_initialized && m_data_size > 0) {
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(m_data, m_data_size);
        if (raw_adv_ret){
            ESP_LOGE(TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
    }
}

void SimpleRadioImpl::setData(const uint8_t *data, size_t len) {
    if(len == 0) {
        m_data_size = 0;
        return;
    }

    if(len >= 30) {
        ESP_LOGW(TAG, "The simple radio data can be only 30 bytes long.");
        len = 30;
    }

    memcpy(m_data+1, data, len);
    m_data_size = len+1;

    if(m_initialized) {
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(m_data, m_data_size);
        if (raw_adv_ret){
            ESP_LOGE(TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
    }
}
