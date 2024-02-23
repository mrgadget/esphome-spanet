/**
 * spanet.cpp
 *
 * Implementation of esphome-spanet
 *
 * Author: Ben Nuttall @mrgadgetnz on Github
 * Last Updated: 2024-02-15
 * License: BSD
 */

#include "spanet.h"
using namespace esphome;

/**
 * Create a new Spanet object
 *
 * Args:
 *   hw_serial: pointer to an Arduino HardwareSerial instance
 *   poll_interval: polling interval in milliseconds
 */
Spanet::Spanet(
        HardwareSerial* hw_serial,
        uint32_t poll_interval
) :
    PollingComponent{poll_interval}, // member initializers list
    hw_serial_{hw_serial}
{
    this->traits_.set_supports_action(true);
    this->traits_.set_supports_current_temperature(true);
    this->traits_.set_supports_two_point_target_temperature(false);
    this->traits_.set_visual_min_temperature(SPANET_MIN_TEMPERATURE);
    this->traits_.set_visual_max_temperature(SPANET_MAX_TEMPERATURE);
    this->traits_.set_visual_temperature_step(SPANET_TEMPERATURE_STEP);
}

void Spanet::check_logger_conflict_() {
#ifdef USE_LOGGER
    if (this->get_hw_serial_() == logger::global_logger->get_hw_serial()) {
        ESP_LOGW(TAG, "  You're using the same serial port for logging"
                " and the Spanet component. Please disable"
                " logging over the serial port by setting"
                " logger:baud_rate to 0.");
    }
#endif
}

void Spanet::setup() {
    // This will be called by App.setup()
    ESP_LOGV(TAG, "setup!");
    this->banner();
    // ESP_LOGI(TAG, "Setting up UART...");
    // if (!this->hw_serial_) {
    //     ESP_LOGI(
    //             TAG,
    //             "No HardwareSerial was provided. "
    //             "Software serial ports are unsupported by this component."
    //     );
    //     this->mark_failed();
    //     return;
    // }
    // this->check_logger_conflict_();

    ESP_LOGI(TAG, "Intializing new SpaNetController object.");
    this->spa = new SpaNetController();
    this->spa->subscribeUpdate([this](int value) {
            handleUpdate(value);
    });

    this->current_temperature = NAN;
    this->target_temperature = NAN;
    this->fan_mode = climate::CLIMATE_FAN_OFF;

    ESP_LOGCONFIG(
            TAG,
            "hw_serial(%p) is &Serial(%p)? %s",
            this->get_hw_serial_(),
            &Serial,
            YESNO(this->get_hw_serial_() == &Serial)
    );

    // create various setpoint persistence:
    heat_storage = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() + 2);

    // load values from storage:
    heat_setpoint = load(heat_storage);

    this->dump_config();
}

void Spanet::update() {
    // This will be called every "update_interval" milliseconds.
    // Response data is processed in the callback below
    this->spa->pollStatus();
    this->spa->tick();

    
}

void Spanet::handleUpdate(int data) {
    ESP_LOGI(TAG, "waterTemp: %f", this->spa->getWaterTemp());

    // ESP_LOGI(TAG, "poll: %s", this->spa->pollStatus() ? "true" : "false");
    // ESP_LOGI(TAG, "debug: %s", this->spa->getDebug());
    // ESP_LOGI(TAG, "pump 1: %s", this->spa->pumpInstalled(1) ? "true" : "false");
    // ESP_LOGI(TAG, "pump 2: %s", this->spa->pumpInstalled(2) ? "true" : "false");
    // ESP_LOGI(TAG, "pump 3: %s", this->spa->pumpInstalled(3) ? "true" : "false");
    // ESP_LOGI(TAG, "model: %s", this->spa->getModel());
    // ESP_LOGI(TAG, "status: %s", this->spa->getStatus());
    // ESP_LOGI(TAG, "waterTemp: %f", this->spa->getWaterTemp());
    // ESP_LOGI(TAG, "setTemp: %f", this->spa->getWaterTempSetPoint());
    // ESP_LOGI(TAG, "serial_rx: %s", this->spa->getRawRx());

    SpaNetController* spa = this->spa;
    Pump* pump = spa->getPump(1);

    //ESP_LOGI(TAG, "pump_mode: %d", pump->getOperatingMode());

    /*
     * ******** Update status. ********
     */
    switch (pump->getOperatingMode())
    {
        case 0:
            ESP_LOGV(TAG, "Mode OFF");
            this->mode = climate::CLIMATE_MODE_OFF;
            this->action = climate::CLIMATE_ACTION_OFF;
            break;
        case 1:
            ESP_LOGV(TAG, "Mode AUTO");
            this->mode = climate::CLIMATE_MODE_HEAT;
            this->fan_mode = climate::CLIMATE_FAN_AUTO;
            break;
        case 2:
            ESP_LOGV(TAG, "Mode LOW");
            this->mode = climate::CLIMATE_MODE_HEAT;
            this->fan_mode = climate::CLIMATE_FAN_LOW;
            break;
        case 3:
            ESP_LOGV(TAG, "Mode HIGH");
            this->mode = climate::CLIMATE_MODE_HEAT;
            this->fan_mode = climate::CLIMATE_FAN_HIGH;
            break;
        case 4:
            ESP_LOGV(TAG, "Mode AUTO");
            this->mode = climate::CLIMATE_MODE_HEAT;
            this->fan_mode = climate::CLIMATE_FAN_AUTO;
            break;
        default:
            break;
    }

    /*
     * ******** Update current action. ********
     */
    if (this->mode == climate::CLIMATE_MODE_OFF) {
        this->action = climate::CLIMATE_ACTION_OFF;
    } else {
        if (spa->isHeatingOn()) {
            this->action = climate::CLIMATE_ACTION_HEATING;
        } else {
            this->action = climate::CLIMATE_ACTION_IDLE;
        }
    }
    
    /*
     * ******** Update temperatures. ********
     */
    this->target_temperature = spa->getWaterTempSetPoint();
    this->current_temperature = spa->getWaterTemp();

    /*
     * ******** Publish state back to ESPHome. ********
     */
    this->publish_state();

#ifndef USE_CALLBACKS
    //this->hpSettingsChanged();
    //heatpumpStatus currentStatus = hp->getStatus();
    //this->hpStatusChanged(currentStatus);
#endif
}

void Spanet::set_baud_rate(int baud) {
    this->baud_ = baud;
}

/**
 * Get our supported traits.
 *
 * Note:
 * Many of the following traits are only available in the 1.5.0 dev train of
 * ESPHome, particularly the Dry operation mode, and several of the fan modes.
 *
 * Returns:
 *   This class' supported climate::ClimateTraits.
 */
climate::ClimateTraits Spanet::traits() {
    return traits_;
}

/**
 * Modify our supported traits.
 *
 * Returns:
 *   A reference to this class' supported climate::ClimateTraits.
 */
climate::ClimateTraits& Spanet::config_traits() {
    return traits_;
}

/**
 * Implement control of a Spanet controller.
 *
 * Maps HomeAssistant/ESPHome modes to SpaNet modes.
 */
void Spanet::control(const climate::ClimateCall &call) {
    SpaNetController* spa = this->spa;

    bool updated = false;
    bool has_mode = call.get_mode().has_value();
    bool has_temp = call.get_target_temperature().has_value();
    bool has_fan = call.get_fan_mode().has_value();
    if (has_mode) {
        //ESP_LOGI(TAG, "Setting mode...");
        ESP_LOGV(TAG, "Requested pump mode is %d", *call.get_mode());
        this->mode = *call.get_mode();

        switch (this->mode) {
            case climate::CLIMATE_MODE_HEAT:
                this->fan_mode = climate::CLIMATE_FAN_AUTO;
                this->action = climate::CLIMATE_ACTION_IDLE;
                spa->setPumpOperating(2, 4);
                updated = true;
                break;
            case climate::CLIMATE_MODE_OFF:
            default:
                this->fan_mode = climate::CLIMATE_FAN_OFF;
                this->action = climate::CLIMATE_ACTION_OFF;
                spa->setPumpOperating(2, 0);
                updated = true;
                break;
        }
    }

    if (has_temp) {
        ESP_LOGI(TAG, "Requested target temp: %.1f", *call.get_target_temperature());
        spa->setWaterTempSetPoint(*call.get_target_temperature());
        this->target_temperature = *call.get_target_temperature();
        updated = true;
    }

    //const char* FAN_MAP[6] = {"AUTO", "QUIET", "1", "2", "3", "4"};
    if (has_fan) {
        //ESP_LOGV(TAG, "Requested fan mode is %d", *call.get_fan_mode());
        this->fan_mode = *call.get_fan_mode();
        switch(*call.get_fan_mode()) {
            case climate::CLIMATE_FAN_OFF:
                ESP_LOGI(TAG, "Requested fan mode is OFF");
                this->mode = climate::CLIMATE_MODE_OFF;
                this->action = climate::CLIMATE_ACTION_OFF;
                spa->setPump2Operating(0);
                updated = true;
                break;
            case climate::CLIMATE_FAN_LOW:
                ESP_LOGI(TAG, "Requested fan mode is LOW");
                this->mode = climate::CLIMATE_MODE_HEAT;
                this->action = climate::CLIMATE_ACTION_IDLE;
                spa->setPump2Operating(3);
                updated = true;
                break;
            case climate::CLIMATE_FAN_HIGH:
                ESP_LOGI(TAG, "Requested fan mode is HIGH");
                this->mode = climate::CLIMATE_MODE_HEAT;
                this->action = climate::CLIMATE_ACTION_IDLE;
                spa->setPump2Operating(2);
                updated = true;
                break;
            case climate::CLIMATE_FAN_ON:
            case climate::CLIMATE_FAN_AUTO:
            default:
                ESP_LOGI(TAG, "Requested fan mode is AUTO");
                this->mode = climate::CLIMATE_MODE_HEAT;
                this->action = climate::CLIMATE_ACTION_IDLE;
                spa->setPump2Operating(4);
                updated = true;
                break;
        }
    }

    ESP_LOGD(TAG, "control - Was spa updated? %s", YESNO(updated));

    // send the update back to esphome:
    this->publish_state();
    // and the spa:
    spa->tick();
}

/**
 * The ESP only has a few bytes of rtc storage, so instead
 * of storing floats directly, we'll store the number of
 * TEMPERATURE_STEPs from MIN_TEMPERATURE.
 **/
void Spanet::save(float value, ESPPreferenceObject& storage) {
    uint8_t steps = (value - SPANET_MIN_TEMPERATURE) / SPANET_TEMPERATURE_STEP;
    storage.save(&steps);
}

optional<float> Spanet::load(ESPPreferenceObject& storage) {
    uint8_t steps = 0;
    if (!storage.load(&steps)) {
        return {};
    }
    return SPANET_MIN_TEMPERATURE + (steps * SPANET_TEMPERATURE_STEP);
}

void Spanet::dump_config() {
    this->banner();
    ESP_LOGI(TAG, "  Supports HEAT: %s", YESNO(true));
    // ESP_LOGI(TAG, "  Supports COOL: %s", YESNO(true));
    // ESP_LOGI(TAG, "  Supports AWAY mode: %s", YESNO(false));
    ESP_LOGI(TAG, "  Saved heat: %.1f", heat_setpoint.value_or(-1));
    // ESP_LOGI(TAG, "  Saved cool: %.1f", cool_setpoint.value_or(-1));
    // ESP_LOGI(TAG, "  Saved auto: %.1f", auto_setpoint.value_or(-1));
}

void Spanet::dump_state() {
    LOG_CLIMATE("", "Spanet Climate", this);
    ESP_LOGI(TAG, "HELLO");
}
