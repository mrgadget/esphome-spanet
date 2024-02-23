/**
 * spanet.h
 *
 * Header file for esphome-spanet
 *
 * Author: Ben Nuttall @mrgadgetnz on Github
 * Last Updated: 2024-02-10
 * License: BSD
 *
 * Requirements:
 * - ESPHome
 */

#define USE_CALLBACKS

#include "esphome.h"
#include "esphome/core/preferences.h"

#include <functional>

// #include "HeatPump.h"
using namespace esphome;
using namespace std::placeholders;

#ifndef SPANET_H
#define SPANET_H

static const char* TAG = "Spanet"; // Logging tag

static const char* SPANET_VERSION = "0.0.4";

/* If polling interval is greater than 9 seconds, the HeatPump
library reconnects, but doesn't then follow up with our data request.*/
static const uint32_t SPANET_POLL_INTERVAL_DEFAULT = 60000; // in milliseconds, 0 < X <= 9000
static const uint8_t SPANET_MIN_TEMPERATURE = 16; // degrees C,
static const uint8_t SPANET_MAX_TEMPERATURE = 42; // degrees C,
static const float   SPANET_TEMPERATURE_STEP = 0.1; // temperature setting step, in degrees C

class Spanet : public PollingComponent, public climate::Climate {

    public:
        /**
         * Create a new Spanet object
         *
         * Args:
         *   hw_serial: pointer to an Arduino HardwareSerial instance
         *   poll_interval: polling interval in milliseconds
         */
        Spanet(
            HardwareSerial* hw_serial,
            uint32_t poll_interval=SPANET_POLL_INTERVAL_DEFAULT
        );

        // Print a banner with library information.
        void banner() {
            ESP_LOGI(TAG, "ESPHome SpaNet version %s", SPANET_VERSION);
        }

        // Set the baud rate. Must be called before setup() to have any effect.
        void set_baud_rate(int);

        // print the current configuration
        void dump_config() override;

        // Set up the component, initializing the HeatPump object.
        void setup() override;

        // This is called every poll_interval.
        void update() override;

        // handle a change in settings as detected by the SpaNetController library.
        void dataFromSpa(SpaNetController* spa);

        void handleUpdate(int data);

        // Configure the climate object with traits that we support.
        climate::ClimateTraits traits() override;

        // Get a mutable reference to the traits that we support.
        climate::ClimateTraits& config_traits();

        // Debugging function to print the object's state.
        void dump_state();

        // Handle a request from the user to change settings.
        void control(const climate::ClimateCall &call) override;

    protected:
        // HeatPump object using the underlying Arduino library.
        //HeatPump* hp;
        SpaNetController* spa;

        // The ClimateTraits supported by this HeatPump.
        climate::ClimateTraits traits_;

        //Accessor method for the HardwareSerial pointer
        HardwareSerial* get_hw_serial_() {
            return this->hw_serial_;
        }

        //Print a warning message if we're using the sole hardware UART on an
        //ESP8266 or UART0 on ESP32
        void check_logger_conflict_();

        // various prefs to save mode-specific temperatures, akin to how the IR
        // remote works.
        ESPPreferenceObject cool_storage;
        ESPPreferenceObject heat_storage;
        ESPPreferenceObject auto_storage;

        optional<float> heat_setpoint;

        static void save(float value, ESPPreferenceObject& storage);
        static optional<float> load(ESPPreferenceObject& storage);

    private:
        // Retrieve the HardwareSerial pointer from friend and subclasses.
        HardwareSerial *hw_serial_;
        int baud_ = 0;
};

#endif
