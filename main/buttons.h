#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Button IDs for the 10 push buttons
typedef enum {
    BTN_ROTARY = 0,      // Rotary encoder button (GPIO35, separate)
    BTN_BAND_UM,         // Button 2 (ADC)
    BTN_BAND_VF,         // Button 3 (ADC)
    BTN_STATION_1,       // Button 4 (ADC)
    BTN_STATION_2,       // Button 5 (ADC)
    BTN_STATION_3,       // Button 6 (ADC)
    BTN_STATION_4,       // Button 7 (ADC)
    BTN_STATION_5,       // Button 8 (ADC)
    BTN_DOWN,            // Button 9 (ADC)
    BTN_UP,              // Button 10 (ADC)
    BTN_NONE = 0xFF
} button_id_t;


// Button event types
typedef enum {
    BTN_EVENT_PRESS = 0,
    BTN_EVENT_RELEASE = 1,
    BTN_EVENT_LONG_PRESS = 2,
    BTN_EVENT_RELEASE_AFTER_LONG = 3,
    BTN_EVENT_REPEAT = 4,
    BTN_EVENT_ROTARY_CW = 5,
    BTN_EVENT_ROTARY_CCW = 6
} button_event_type_t;

// Button event structure
typedef struct {
    button_id_t button;
    button_event_type_t event;
    uint32_t timestamp;
} button_event_t;

// Button event callback
typedef void (*button_callback_t)(button_event_t event);

/**
 * @brief Initialize button input system
 * @param adc_pin GPIO pin for ADC button reading (10 buttons via voltage divider)
 * @param rotary_clk GPIO pin for rotary encoder CLK (A phase)
 * @param rotary_dt GPIO pin for rotary encoder DT (B phase)
 * @param rotary_btn GPIO pin for rotary encoder button (included in ADC chain)
 * @param callback Function to call when button events occur
 * @return ESP_OK on success
 */
esp_err_t buttons_init(int adc_pin, int rotary_clk, int rotary_dt, 
                       int rotary_sw, button_callback_t callback);

/**
 * @brief Get current rotary encoder position
 * @return Current position value (relative counter)
 */
int32_t buttons_get_rotary_position(void);

/**
 * @brief Reset rotary encoder position to zero
 */
void buttons_reset_rotary_position(void);

#ifdef __cplusplus
}
#endif

#endif // BUTTONS_H
