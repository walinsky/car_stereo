#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "car_stereo_state.h"

// Display configuration for 16x2 character LCD (SLC1602A3)
#define DISPLAY_COLS    16
#define DISPLAY_ROWS    2

// I2C configuration
#define LCD_I2C_PORT    I2C_NUM_0
#define LCD_I2C_SDA     21
#define LCD_I2C_SCL     22
#define LCD_I2C_FREQ    100000  // 100kHz
#define LCD_I2C_ADDR    0x27    // PCF8574 default address

// Display mode enumeration
typedef enum {
    DISPLAY_MODE_OFF,
    DISPLAY_MODE_RADIO,
    DISPLAY_MODE_BLUETOOTH,
    DISPLAY_MODE_PHONE_CALL,
    DISPLAY_MODE_PHONEBOOK
} display_mode_t;

// Display state structure
typedef struct {
    display_mode_t mode;
    char line1[17];  // 16 chars + null terminator
    char line2[17];
    uint8_t volume;
    bool playing;
    bool connected;
} display_state_t;

void display_test_simple(void);

/**
 * @brief Initialize the I2C LCD display
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_init(void);

/**
 * @brief Update the display with current state
 * 
 * @param state Pointer to display state structure
 */
void display_update(const display_state_t *state);

/**
 * @brief Show splash screen on boot
 */
void display_show_splash(void);

/**
 * @brief Clear the display
 */
void display_clear(void);

/**
 * @brief Display a notification message
 * 
 * @param line1 First line of text (max 16 chars)
 * @param line2 Second line of text (max 16 chars)
 * @param duration_ms Duration to show message in milliseconds
 */
void display_notification(const char *line1, const char *line2, uint16_t duration_ms);

/**
 * @brief Control the backlight
 * 
 * @param on true to turn backlight on, false to turn off
 */
void display_set_backlight(bool on);

void display_handle_notification(display_notification_t notification);

void sanitize_for_lcd(char *dest, const char *src, size_t max_len);

#endif // DISPLAY_H
