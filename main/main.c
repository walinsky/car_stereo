#include "a2dpSinkHfpHf.h"
#include "car_stereo_state.h"
#include "buttons.h"

// Add these missing includes:
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include <string.h>
#include "display.h"
#include "driver/i2c.h" // for scanning i2c bus. dev.
// debugging non standard characters
#include <stdio.h>
#include <ctype.h>

#define TAG "CAR_STEREO"

// GPIO pin definitions
#define ADC_BUTTON_PIN      36    // GPIO26 (ADC1_CH6) - Button network
#define ROTARY_CLK_PIN      25    // GPIO35 - Encoder CLK
#define ROTARY_DT_PIN       33    // GPIO33 - Encoder DT
#define ROTARY_SW_PIN       32    // GPIO32 - Encoder button (separate)

// Forward declarations
static void button_event_callback(button_event_t event);
static void mode_change_callback(stereo_mode_t old_mode, stereo_mode_t new_mode);
static void display_handler(display_notification_t notification);

void debug_dump_ascii_and_hex(const char *label, const char *s)
{
    printf("%s: \"", label);
    for (const char *p = s; *p; ++p)
        putchar(isprint((unsigned char)*p) ? *p : '.');
    printf("\"\n%s HEX:", label);
    for (const char *p = s; *p && (p - s < 80); ++p)
        printf(" %02X", (unsigned char)*p);
    printf("\n");
}

static void bt_connection_callback(bool connected, const uint8_t *remote_bda)
{
    if (connected) {
        ESP_LOGI(TAG, "=== BLUETOOTH DEVICE CONNECTED ===");
        if (remote_bda) {
            ESP_LOGI(TAG, "Address: %02X:%02X:%02X:%02X:%02X:%02X",
                     remote_bda[0], remote_bda[1], remote_bda[2],
                     remote_bda[3], remote_bda[4], remote_bda[5]);
        }
        // Notify state machine
        stereo_state_bt_device_connected(remote_bda);
    } else {
        ESP_LOGI(TAG, "=== BLUETOOTH DEVICE DISCONNECTED ===");
        stereo_state_bt_device_disconnected(NULL);
    }
}

static void a2dp_audio_state_callback(bool streaming)
{
    ESP_LOGI(TAG, "=== A2DP AUDIO %s ===", streaming ? "STARTED" : "STOPPED");
    
    if (streaming) {
        // Audio started streaming - switch to Bluetooth mode if not in a call
        stereo_mode_t current_mode = stereo_state_get_mode();
        
        if (current_mode != MODE_PHONE_CALL && current_mode != MODE_BLUETOOTH) {
            ESP_LOGI(TAG, "Auto-switching to Bluetooth mode");
            stereo_state_set_mode(MODE_BLUETOOTH);
        }
        
        // Notify state machine that audio is playing
        stereo_state_a2dp_streaming(true);
    } else {
        // Audio stopped
        stereo_state_a2dp_streaming(false);
    }
}


static void hfp_call_state_callback(bool call_active, int call_state)
{
    ESP_LOGI(TAG, "=== HFP CALL STATE: %s (state=%d) ===", 
             call_active ? "ACTIVE" : "IDLE", call_state);
    
    if (call_active) {
        // Switch to phone call mode
        // stereo_state_set_mode(MODE_PHONE_CALL);
    } else {
        // Return to previous mode
        // stereo_state_set_mode(MODE_BLUETOOTH);
    }
}

// AVRCP metadata callback - shows track info when music changes
static void avrcp_metadata_callback(const bt_avrc_metadata_t *metadata)
{
    if (!metadata) return;
    
    debug_dump_ascii_and_hex("ARTIST RAW", metadata->artist);
    debug_dump_ascii_and_hex("TITLE RAW", metadata->title);
    debug_dump_ascii_and_hex("ALBUM RAW", metadata->album);

    char title_clean[128];
    char artist_clean[64];
    char album_clean[64];
    
    // Sanitize UTF-8 to ASCII
    sanitize_for_lcd(title_clean, metadata->title, sizeof(title_clean));
    sanitize_for_lcd(artist_clean, metadata->artist, sizeof(artist_clean));
    sanitize_for_lcd(album_clean, metadata->album, sizeof(album_clean));
    
    ESP_LOGI(TAG, "=== TRACK CHANGED ===");
    ESP_LOGI(TAG, "♫ Title:  %s", title_clean);
    ESP_LOGI(TAG, "♪ Artist: %s", artist_clean);
    ESP_LOGI(TAG, "⊙ Album:  %s", album_clean);
    
    // Use the existing public API to update display
    stereo_state_a2dp_metadata(
        title_clean,
        artist_clean,
        album_clean
    );
}
// GAP event handler - called by bt_gap.c
static void gap_event_handler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Device connected: %02X:%02X:%02X:%02X:%02X:%02X",
                         param->auth_cmpl.bda[0], param->auth_cmpl.bda[1],
                         param->auth_cmpl.bda[2], param->auth_cmpl.bda[3],
                         param->auth_cmpl.bda[4], param->auth_cmpl.bda[5]);
                stereo_state_bt_device_connected(param->auth_cmpl.bda);
            }
            break;
            
        case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
            ESP_LOGI(TAG, "Device disconnected");
            // Note: We don't have the MAC in this event, pass NULL
            stereo_state_bt_device_disconnected(NULL);
            break;
            
        default:
            break;
    }
}

void i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address: 0x%02X", addr);
        }
    }
}

void app_main(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32 Car Stereo Starting...");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "Initializing display...");
    ESP_ERROR_CHECK(display_init());
    
    // Initialize state machine
    stereo_config_t config = {
        .display_handler = display_handle_notification,
        .on_mode_change = mode_change_callback
    };
    ESP_ERROR_CHECK(stereo_state_init(&config));
    
    // Initialize button handler
    ESP_ERROR_CHECK(buttons_init(ADC_BUTTON_PIN, ROTARY_CLK_PIN,
                                 ROTARY_DT_PIN, ROTARY_SW_PIN,
                                 button_event_callback));
    ESP_LOGI(TAG, "Buttons initialized on ADC GPIO%d", ADC_BUTTON_PIN);
    ESP_LOGI(TAG, "Rotary encoder: CLK=GPIO%d, DT=GPIO%d", ROTARY_CLK_PIN, ROTARY_DT_PIN);
    
    // Initialize Bluetooth A2DP/HFP component
    ESP_ERROR_CHECK(a2dpSinkHfpHf_init(NULL));
    
    // Register connection/audio callbacks
    a2dp_sink_hfp_hf_register_connection_cb(bt_connection_callback);
    a2dp_sink_hfp_hf_register_audio_state_cb(a2dp_audio_state_callback);
    a2dp_sink_hfp_hf_register_call_state_cb(hfp_call_state_callback);
    a2dpSinkHfpHf_register_avrc_metadata_callback(avrcp_metadata_callback);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Car Stereo Ready!");
    ESP_LOGI(TAG, "========================================");
    
    // Main loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        // int clk = gpio_get_level(ROTARY_CLK_PIN);
        // int dt  = gpio_get_level(ROTARY_DT_PIN);
        // int sw  = gpio_get_level(ROTARY_SW_PIN);
        // ESP_LOGI("ROTARY_DBG", "CLK=%d, DT=%d, SW=%d", clk, dt, sw);
        // vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Button event callback
static void button_event_callback(button_event_t event)
{
    // ESP_LOGI(TAG, "Button event: btn=%d, type=%d", event.button, event.event);
    // if (event.event == BTN_EVENT_PRESS || event.event == BTN_EVENT_LONG_PRESS) {
    //     ESP_LOGI(TAG, "Button event: btn=%d, type=%d", event.button, event.event);
    // }
    // Forward to state machine
    stereo_state_handle_button(event);
}

// Mode change callback
static void mode_change_callback(stereo_mode_t old_mode, stereo_mode_t new_mode)
{
    const char *mode_names[] = {"OFF", "RADIO", "BLUETOOTH", "PHONE_CALL", "PHONEBOOK"};
    
    ESP_LOGI(TAG, "Mode changed: %s -> %s", 
             mode_names[old_mode], mode_names[new_mode]);
    
    // Handle mode-specific initialization/cleanup here
    switch (new_mode) {
        case MODE_RADIO:
            // Enable radio tuner, etc.
            break;
            
        case MODE_BLUETOOTH:
            // Ensure BT audio is ready
            break;
            
        case MODE_PHONE_CALL:
            // Switch audio routing to call
            break;
            
        default:
            break;
    }
}
