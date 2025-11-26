/*
 * Car Stereo State Machine
 * Manages all state transitions, button handling, and business logic
 */

#include "car_stereo_state.h"
#include "a2dpSinkHfpHf.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "STEREO_STATE"
#define NVS_NAMESPACE "car_stereo"

// NVS Keys
#define NVS_KEY_POWER_ON        "power_on"
#define NVS_KEY_MODE            "mode"
#define NVS_KEY_RADIO_BAND      "radio_band"
#define NVS_KEY_RADIO_FREQ      "radio_freq"
#define NVS_KEY_RADIO_VOL       "radio_vol"
#define NVS_KEY_A2DP_VOL        "a2dp_vol"
#define NVS_KEY_HFP_SPK_VOL     "hfp_spk_vol"
#define NVS_KEY_HFP_MIC_VOL     "hfp_mic_vol"
#define NVS_KEY_PRESET_FM_1     "preset_fm_1"
#define NVS_KEY_PRESET_FM_2     "preset_fm_2"
#define NVS_KEY_PRESET_FM_3     "preset_fm_3"
#define NVS_KEY_PRESET_FM_4     "preset_fm_4"
#define NVS_KEY_PRESET_FM_5     "preset_fm_5"
#define NVS_KEY_PRESET_AM_1     "preset_am_1"
#define NVS_KEY_PRESET_AM_2     "preset_am_2"
#define NVS_KEY_PRESET_AM_3     "preset_am_3"
#define NVS_KEY_PRESET_AM_4     "preset_am_4"
#define NVS_KEY_PRESET_AM_5     "preset_am_5"
#define NVS_KEY_BT_DEV_COUNT    "bt_dev_cnt"
#define NVS_KEY_BT_DEV_PREFIX   "bt_dev_"

#define MAX_BT_DEVICES 5
#define STATION_TUNE_DELAY_MS 2000

typedef struct {
    uint8_t mac_addr[6];
    uint8_t a2dp_volume;      // A2DP volume for this device
    uint8_t hfp_speaker_volume; // HFP speaker volume for this device
    uint8_t hfp_mic_volume;    // HFP mic volume for this device
    bool valid;
} bt_device_settings_t;

// Global state
static bool g_powered_on = false;
static stereo_mode_t g_current_mode = MODE_OFF;
static stereo_mode_t g_mode_before_call = MODE_RADIO;
static stereo_mode_t g_mode_before_phonebook = MODE_RADIO;
static radio_state_t g_radio_state;
static a2dp_state_t g_a2dp_state;
static hfp_state_t g_hfp_state;
static phonebook_state_t g_phonebook_state;
static stereo_config_t g_config;
static radio_band_t g_current_band = RADIO_BAND_FM;
static bt_device_settings_t g_bt_devices[MAX_BT_DEVICES];
static uint8_t g_current_bt_device_mac[6] = {0};
static bool g_browsing_stations = false;
static uint8_t g_browsing_station_idx = 0;
static float g_browsing_station_freq = 87.5;
static TaskHandle_t g_station_tune_timer = NULL;
static bool g_voice_command_active = false;
static char g_caller_id[64] = {0};
static bool g_voice_recognition_active = false;

// Forward declarations
static void save_to_nvs(void);
static void on_bt_volume_changed(bt_volume_target_t target, uint8_t new_volume);

// ============================================================================
// NVS PERSISTENCE - GENERAL STATE
// ============================================================================

static void save_to_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    
    if (err == ESP_OK) {
        nvs_set_u8(nvs_handle, NVS_KEY_POWER_ON, g_powered_on ? 1 : 0);
        nvs_set_u8(nvs_handle, NVS_KEY_MODE, (uint8_t)g_current_mode);
        nvs_set_u8(nvs_handle, NVS_KEY_RADIO_BAND, (uint8_t)g_current_band);
        nvs_set_u32(nvs_handle, NVS_KEY_RADIO_FREQ, (uint32_t)(g_radio_state.frequency * 100));
        nvs_set_u8(nvs_handle, NVS_KEY_RADIO_VOL, g_radio_state.volume);
        
        // A2DP volume
        nvs_set_u8(nvs_handle, NVS_KEY_A2DP_VOL, g_a2dp_state.volume);
        
        // HFP volumes
        nvs_set_u8(nvs_handle, NVS_KEY_HFP_SPK_VOL, g_hfp_state.speaker_volume);
        nvs_set_u8(nvs_handle, NVS_KEY_HFP_MIC_VOL, g_hfp_state.mic_volume);
        
        // FM presets
        nvs_set_u32(nvs_handle, NVS_KEY_PRESET_FM_1, (uint32_t)(g_radio_state.preset_freq[RADIO_BAND_FM][0] * 100));
        nvs_set_u32(nvs_handle, NVS_KEY_PRESET_FM_2, (uint32_t)(g_radio_state.preset_freq[RADIO_BAND_FM][1] * 100));
        nvs_set_u32(nvs_handle, NVS_KEY_PRESET_FM_3, (uint32_t)(g_radio_state.preset_freq[RADIO_BAND_FM][2] * 100));
        nvs_set_u32(nvs_handle, NVS_KEY_PRESET_FM_4, (uint32_t)(g_radio_state.preset_freq[RADIO_BAND_FM][3] * 100));
        nvs_set_u32(nvs_handle, NVS_KEY_PRESET_FM_5, (uint32_t)(g_radio_state.preset_freq[RADIO_BAND_FM][4] * 100));
        
        // AM presets
        nvs_set_u32(nvs_handle, NVS_KEY_PRESET_AM_1, (uint32_t)(g_radio_state.preset_freq[RADIO_BAND_AM][0] * 100));
        nvs_set_u32(nvs_handle, NVS_KEY_PRESET_AM_2, (uint32_t)(g_radio_state.preset_freq[RADIO_BAND_AM][1] * 100));
        nvs_set_u32(nvs_handle, NVS_KEY_PRESET_AM_3, (uint32_t)(g_radio_state.preset_freq[RADIO_BAND_AM][2] * 100));
        nvs_set_u32(nvs_handle, NVS_KEY_PRESET_AM_4, (uint32_t)(g_radio_state.preset_freq[RADIO_BAND_AM][3] * 100));
        nvs_set_u32(nvs_handle, NVS_KEY_PRESET_AM_5, (uint32_t)(g_radio_state.preset_freq[RADIO_BAND_AM][4] * 100));
        
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGD(TAG, "State saved to NVS");
    }
}

static void load_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    
    if (err == ESP_OK) {
        uint8_t u8val;
        uint32_t u32val;
        
        if (nvs_get_u8(nvs_handle, NVS_KEY_POWER_ON, &u8val) == ESP_OK) g_powered_on = (u8val != 0);
        if (nvs_get_u8(nvs_handle, NVS_KEY_MODE, &u8val) == ESP_OK) g_current_mode = (stereo_mode_t)u8val;
        if (nvs_get_u8(nvs_handle, NVS_KEY_RADIO_BAND, &u8val) == ESP_OK) {
            g_current_band = (radio_band_t)u8val;
            g_radio_state.band = g_current_band;
        }
        if (nvs_get_u32(nvs_handle, NVS_KEY_RADIO_FREQ, &u32val) == ESP_OK) g_radio_state.frequency = u32val / 100.0f;
        if (nvs_get_u8(nvs_handle, NVS_KEY_RADIO_VOL, &u8val) == ESP_OK) g_radio_state.volume = u8val;
        
        // FM presets
        if (nvs_get_u32(nvs_handle, NVS_KEY_PRESET_FM_1, &u32val) == ESP_OK) g_radio_state.preset_freq[RADIO_BAND_FM][0] = u32val / 100.0f;
        if (nvs_get_u32(nvs_handle, NVS_KEY_PRESET_FM_2, &u32val) == ESP_OK) g_radio_state.preset_freq[RADIO_BAND_FM][1] = u32val / 100.0f;
        if (nvs_get_u32(nvs_handle, NVS_KEY_PRESET_FM_3, &u32val) == ESP_OK) g_radio_state.preset_freq[RADIO_BAND_FM][2] = u32val / 100.0f;
        if (nvs_get_u32(nvs_handle, NVS_KEY_PRESET_FM_4, &u32val) == ESP_OK) g_radio_state.preset_freq[RADIO_BAND_FM][3] = u32val / 100.0f;
        if (nvs_get_u32(nvs_handle, NVS_KEY_PRESET_FM_5, &u32val) == ESP_OK) g_radio_state.preset_freq[RADIO_BAND_FM][4] = u32val / 100.0f;
        
        // AM presets
        if (nvs_get_u32(nvs_handle, NVS_KEY_PRESET_AM_1, &u32val) == ESP_OK) g_radio_state.preset_freq[RADIO_BAND_AM][0] = u32val / 100.0f;
        if (nvs_get_u32(nvs_handle, NVS_KEY_PRESET_AM_2, &u32val) == ESP_OK) g_radio_state.preset_freq[RADIO_BAND_AM][1] = u32val / 100.0f;
        if (nvs_get_u32(nvs_handle, NVS_KEY_PRESET_AM_3, &u32val) == ESP_OK) g_radio_state.preset_freq[RADIO_BAND_AM][2] = u32val / 100.0f;
        if (nvs_get_u32(nvs_handle, NVS_KEY_PRESET_AM_4, &u32val) == ESP_OK) g_radio_state.preset_freq[RADIO_BAND_AM][3] = u32val / 100.0f;
        if (nvs_get_u32(nvs_handle, NVS_KEY_PRESET_AM_5, &u32val) == ESP_OK) g_radio_state.preset_freq[RADIO_BAND_AM][4] = u32val / 100.0f;
        
        // A2DP volume
        if (nvs_get_u8(nvs_handle, NVS_KEY_A2DP_VOL, &u8val) == ESP_OK)
            g_a2dp_state.volume = u8val;
            
        // HFP volumes
        if (nvs_get_u8(nvs_handle, NVS_KEY_HFP_SPK_VOL, &u8val) == ESP_OK)
            g_hfp_state.speaker_volume = u8val;
        if (nvs_get_u8(nvs_handle, NVS_KEY_HFP_MIC_VOL, &u8val) == ESP_OK)
            g_hfp_state.mic_volume = u8val;
        
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "State loaded - Power: %s, Mode: %d, Band: %s",
                 g_powered_on ? "ON" : "OFF", g_current_mode,
                 g_current_band == RADIO_BAND_FM ? "FM" : "AM");
    } else {
        ESP_LOGI(TAG, "No saved state, using defaults");
        g_powered_on = false;
        g_current_mode = MODE_OFF;
    }
}

// ============================================================================
// NVS PERSISTENCE - BLUETOOTH DEVICE SETTINGS
// ============================================================================

static void save_bt_device_settings(void)
{
    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
        uint8_t count = 0;
        for (int i = 0; i < MAX_BT_DEVICES; i++) {
            if (g_bt_devices[i].valid) {
                count++;
                char key[16];
                snprintf(key, sizeof(key), "%s%d", NVS_KEY_BT_DEV_PREFIX, i);
                
                // Pack into blob: 6 bytes MAC + 3 bytes volumes
                uint8_t blob[9];
                memcpy(blob, g_bt_devices[i].mac_addr, 6);
                blob[6] = g_bt_devices[i].a2dp_volume;
                blob[7] = g_bt_devices[i].hfp_speaker_volume;
                blob[8] = g_bt_devices[i].hfp_mic_volume;
                
                nvs_set_blob(nvs_handle, key, blob, sizeof(blob));
            }
        }
        nvs_set_u8(nvs_handle, NVS_KEY_BT_DEV_COUNT, count);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Saved %d BT device settings", count);
    }
}

static void load_bt_device_settings(void)
{
    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle) == ESP_OK) {
        uint8_t count = 0;
        nvs_get_u8(nvs_handle, NVS_KEY_BT_DEV_COUNT, &count);
        
        for (int i = 0; i < MAX_BT_DEVICES && i < count; i++) {
            char key[16];
            snprintf(key, sizeof(key), "%s%d", NVS_KEY_BT_DEV_PREFIX, i);
            
            uint8_t blob[9];
            size_t len = sizeof(blob);
            if (nvs_get_blob(nvs_handle, key, blob, &len) == ESP_OK) {
                memcpy(g_bt_devices[i].mac_addr, blob, 6);
                g_bt_devices[i].a2dp_volume = blob[6];
                g_bt_devices[i].hfp_speaker_volume = blob[7];
                g_bt_devices[i].hfp_mic_volume = blob[8];
                g_bt_devices[i].valid = true;
            }
        }
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Loaded %d BT device settings", count);
    }
}

static int find_bt_device_by_mac(const uint8_t *mac)
{
    for (int i = 0; i < MAX_BT_DEVICES; i++) {
        if (g_bt_devices[i].valid && memcmp(g_bt_devices[i].mac_addr, mac, 6) == 0) {
            return i;
        }
    }
    return -1;
}

static int find_free_bt_device_slot(void)
{
    for (int i = 0; i < MAX_BT_DEVICES; i++) {
        if (!g_bt_devices[i].valid) return i;
    }
    return 0;
}

// ============================================================================
// DISPLAY NOTIFICATIONS
// ============================================================================

static void send_display_notification(display_notification_type_t type, 
                                      const char *text, const char *subtext,
                                      uint32_t duration_ms, uint8_t priority)
{
    if (g_config.display_handler) {
        display_notification_t notif = {.type = type, .duration_ms = duration_ms, .priority = priority};
        if (text) strncpy(notif.text, text, sizeof(notif.text) - 1);
        if (subtext) strncpy(notif.subtext, subtext, sizeof(notif.subtext) - 1);
        g_config.display_handler(notif);
    }
}

// ============================================================================
// STATION TUNING TIMER
// ============================================================================

static void station_tune_timer_callback(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(STATION_TUNE_DELAY_MS));
    if (g_browsing_stations) {
        ESP_LOGI(TAG, "Tuning to %.1f MHz", g_browsing_station_freq);
        g_radio_state.frequency = g_browsing_station_freq;
        g_browsing_stations = false;
        char freq_str[16];
        snprintf(freq_str, sizeof(freq_str), "%.1f MHz", g_browsing_station_freq);
        send_display_notification(DISPLAY_FREQUENCY, freq_str, "Tuned", 2000, 100);
        save_to_nvs();
    }
    g_station_tune_timer = NULL;
    vTaskDelete(NULL);
}

static void start_station_tune_timer(void)
{
    if (g_station_tune_timer) vTaskDelete(g_station_tune_timer);
    xTaskCreate(station_tune_timer_callback, "tune_timer", 2048, NULL, 5, &g_station_tune_timer);
}

// ============================================================================
// MODE HANDLERS
// ============================================================================

static void handle_radio_mode(button_event_t event)
{
    switch (event.event) {
        case BTN_EVENT_ROTARY_CW:
            // Volume up or station browsing
            if (g_browsing_stations) {
                g_browsing_station_idx = (g_browsing_station_idx + 1) % 20;
                g_browsing_station_freq = 87.5 + (g_browsing_station_idx * 0.2);
                char freq[16];
                snprintf(freq, sizeof(freq), "%.1f MHz", g_browsing_station_freq);
                send_display_notification(DISPLAY_FREQUENCY, freq, "Browsing", 0, 150);
                start_station_tune_timer();
            } else {
                if (g_radio_state.volume < 15) g_radio_state.volume++;
                char vol[8];
                snprintf(vol, sizeof(vol), "%d", g_radio_state.volume);
                send_display_notification(DISPLAY_VOLUME, vol, "Radio", 1000, 120);
                save_to_nvs();
            }
            break;

        case BTN_EVENT_ROTARY_CCW:
            // Volume down or station browsing
            if (g_browsing_stations) {
                g_browsing_station_idx = (g_browsing_station_idx == 0) ? 19 : (g_browsing_station_idx - 1);
                g_browsing_station_freq = 87.5 + (g_browsing_station_idx * 0.2);
                char freq[16];
                snprintf(freq, sizeof(freq), "%.1f MHz", g_browsing_station_freq);
                send_display_notification(DISPLAY_FREQUENCY, freq, "Browsing", 0, 150);
                start_station_tune_timer();
            } else {
                if (g_radio_state.volume > 0) g_radio_state.volume--;
                char vol[8];
                snprintf(vol, sizeof(vol), "%d", g_radio_state.volume);
                send_display_notification(DISPLAY_VOLUME, vol, "Radio", 1000, 120);
                save_to_nvs();
            }
            break;

        case BTN_EVENT_PRESS:
            // Add this case to satisfy -Werror=switch
            if (event.button == BTN_ROTARY && !g_browsing_stations) {
                char freq[16];
                snprintf(freq, sizeof(freq), "%.1f MHz", g_radio_state.frequency);
                send_display_notification(DISPLAY_FREQUENCY, freq, 
                                        g_current_band == RADIO_BAND_FM ? "FM" : "AM", 
                                        2000, 130);
            }
            break;

        case BTN_EVENT_RELEASE:
            // Station presets (buttons 3-7: BTN_STATION_1 through BTN_STATION_5)
            if (event.button >= BTN_STATION_1 && event.button <= BTN_STATION_5) {
                int idx = event.button - BTN_STATION_1;
                float freq = g_radio_state.preset_freq[g_current_band][idx];
                if (freq > 0) {
                    g_radio_state.frequency = freq;
                    char msg[32];
                    snprintf(msg, sizeof(msg), "Station %d: %.1f MHz", idx + 1, freq);
                    send_display_notification(DISPLAY_FREQUENCY, msg, NULL, 2000, 130);
                    save_to_nvs();
                }
            }
            // Seek buttons
            else if (event.button == BTN_UP) {
                send_display_notification(DISPLAY_MODE_CHANGE, "Seeking Up", NULL, 1000, 110);
            }
            else if (event.button == BTN_DOWN) {
                send_display_notification(DISPLAY_MODE_CHANGE, "Seeking Down", NULL, 1000, 110);
            }
            // Rotary button toggles browsing mode
            else if (event.button == BTN_ROTARY) {
                g_browsing_stations = !g_browsing_stations;
                if (g_browsing_stations) {
                    g_browsing_station_freq = g_radio_state.frequency;
                    g_browsing_station_idx = (uint8_t)((g_browsing_station_freq - 87.5) / 0.2);
                    char freq[16];
                    snprintf(freq, sizeof(freq), "%.1f MHz", g_browsing_station_freq);
                    send_display_notification(DISPLAY_FREQUENCY, freq, "Browse Mode", 0, 150);
                }
            }
            break;

        case BTN_EVENT_LONG_PRESS:
            // Save station presets (all 5 buttons now available)
            if (event.button >= BTN_STATION_1 && event.button <= BTN_STATION_5) {
                int idx = event.button - BTN_STATION_1;
                g_radio_state.preset_freq[g_current_band][idx] = g_radio_state.frequency;
                char msg[32];
                snprintf(msg, sizeof(msg), "Station %d Saved", idx + 1);
                send_display_notification(DISPLAY_FREQUENCY, msg, NULL, 2000, 140);
                save_to_nvs();
            }
            break;

        case BTN_EVENT_REPEAT:
        case BTN_EVENT_RELEASE_AFTER_LONG:
            break;
    }
}

static void handle_bluetooth_mode(button_event_t event)
{
    switch (event.event) {
        case BTN_EVENT_PRESS:
            // Handle press if needed
            break;

        case BTN_EVENT_ROTARY_CW:
            if (g_a2dp_state.volume < 15) g_a2dp_state.volume++;
            char vol[8];
            snprintf(vol, sizeof(vol), "%d", g_a2dp_state.volume);
            send_display_notification(DISPLAY_VOLUME, vol, "Bluetooth", 1000, 120);
            a2dpSinkHfpHf_set_a2dp_volume(g_a2dp_state.volume);
            save_to_nvs();
            break;

        case BTN_EVENT_ROTARY_CCW:
            if (g_a2dp_state.volume > 0) g_a2dp_state.volume--;
            char vol2[8];
            snprintf(vol2, sizeof(vol2), "%d", g_a2dp_state.volume);
            send_display_notification(DISPLAY_VOLUME, vol2, "Bluetooth", 1000, 120);
            a2dpSinkHfpHf_set_a2dp_volume(g_a2dp_state.volume);
            save_to_nvs();
            break;

        case BTN_EVENT_RELEASE:
            if (event.button == BTN_ROTARY) {
                // Play/Pause toggle
                if (g_a2dp_state.playing) {
                    // FIXED: Use correct function name
                    a2dpSinkHfpHf_avrc_pause();
                    g_a2dp_state.playing = false;
                    send_display_notification(DISPLAY_MODE_CHANGE, "Paused", NULL, 1000, 110);
                } else {
                    // FIXED: Use correct function name
                    a2dpSinkHfpHf_avrc_play();
                    g_a2dp_state.playing = true;
                    send_display_notification(DISPLAY_MODE_CHANGE, "Playing", NULL, 1000, 110);
                }
            }
            else if (event.button == BTN_UP) {
                // FIXED: Use correct function name
                a2dpSinkHfpHf_avrc_next();
                send_display_notification(DISPLAY_MODE_CHANGE, "Next Track", NULL, 1000, 110);
            }
            else if (event.button == BTN_DOWN) {
                // FIXED: Use correct function name
                a2dpSinkHfpHf_avrc_prev();
                send_display_notification(DISPLAY_MODE_CHANGE, "Previous Track", NULL, 1000, 110);
            }
            break;

        case BTN_EVENT_LONG_PRESS:
            // Voice control removed - now handled by buttons 1 & 2
            break;

        case BTN_EVENT_RELEASE_AFTER_LONG:
            break;

        case BTN_EVENT_REPEAT:
            if (event.button == BTN_UP || event.button == BTN_DOWN) {
                // Volume repeat
                if (event.button == BTN_UP && g_a2dp_state.volume < 15) {
                    g_a2dp_state.volume++;
                } else if (event.button == BTN_DOWN && g_a2dp_state.volume > 0) {
                    g_a2dp_state.volume--;
                }
                char vol3[8];
                snprintf(vol3, sizeof(vol3), "%d", g_a2dp_state.volume);
                send_display_notification(DISPLAY_VOLUME, vol3, "Bluetooth", 1000, 120);
                a2dpSinkHfpHf_set_a2dp_volume(g_a2dp_state.volume);
                save_to_nvs();
            }
            break;
    }
}

static void handle_phone_call_mode(button_event_t event)
{
    switch (event.event) {
        case BTN_EVENT_RELEASE:
            break;
        case BTN_EVENT_LONG_PRESS:
            break;
        case BTN_EVENT_ROTARY_CW:
            if (g_a2dp_state.volume < 15) {
                g_a2dp_state.volume++;
                char vol[8];
                snprintf(vol, sizeof(vol), "%d", g_a2dp_state.volume);
                send_display_notification(DISPLAY_VOLUME, vol, "Call Volume", 1000, 200);
                a2dpSinkHfpHf_set_hfp_speaker_volume(g_hfp_state.speaker_volume);
            }
            break;
            
        case BTN_EVENT_ROTARY_CCW:
            if (g_a2dp_state.volume > 0) {
                g_a2dp_state.volume--;
                char vol[8];
                snprintf(vol, sizeof(vol), "%d", g_a2dp_state.volume);
                send_display_notification(DISPLAY_VOLUME, vol, "Call Volume", 1000, 200);
                a2dpSinkHfpHf_set_hfp_speaker_volume(g_hfp_state.speaker_volume);
            }
            break;
            
        case BTN_EVENT_PRESS:
            if (event.button == BTN_ROTARY) {
                a2dpSinkHfpHf_hangup_call();
                send_display_notification(DISPLAY_CALL_ACTIVE, "Call Ended", NULL, 2000, 250);
                g_current_mode = g_mode_before_call;
                if (g_config.on_mode_change) {
                    g_config.on_mode_change(MODE_PHONE_CALL, g_current_mode);
                }
                save_to_nvs();
            }
            break;

        case BTN_EVENT_RELEASE_AFTER_LONG:
            // Nothing to do - already handled by long press
            break;

        case BTN_EVENT_REPEAT:
            // Handle call volume repeat if needed
            // Volume already handled by rotary CW/CCW
            break;       
    }
}

static void handle_phonebook_mode(button_event_t event)
{
    // Phonebook navigation - simplified placeholder
    // Implement full phonebook browsing based on earlier conversation
}

static void on_bt_volume_changed(bt_volume_target_t target, uint8_t new_volume)
{
    char vol_str[8];
    snprintf(vol_str, sizeof(vol_str), "%d", new_volume);
    
    const char *context = NULL;
    switch (target) {
        case BT_VOLUME_TARGET_A2DP:
            context = "Bluetooth";
            g_a2dp_state.volume = new_volume;  // Update A2DP state
            break;
        case BT_VOLUME_TARGET_HFP_SPEAKER:
            context = "Call Volume";
            g_hfp_state.speaker_volume = new_volume;  // Update HFP speaker state
            break;
        case BT_VOLUME_TARGET_HFP_MIC:
            context = "Mic Volume";
            g_hfp_state.mic_volume = new_volume;  // Update HFP mic state
            break;
        default:
            context = "Volume";
    }
    
    send_display_notification(DISPLAY_VOLUME, vol_str, context, 1000, 120);
}

// ============================================================================
// PUBLIC API
// ============================================================================

esp_err_t stereo_state_init(const stereo_config_t *config)
{
    ESP_LOGI(TAG, "Initializing car stereo state machine");
    
    if (config) {
        memcpy(&g_config, config, sizeof(stereo_config_t));
    }
    
    // Initialize defaults
    g_radio_state.frequency = 87.5;
    g_radio_state.volume = 10;
    g_radio_state.preset_freq[RADIO_BAND_FM][0] = 87.9;
    g_radio_state.preset_freq[RADIO_BAND_FM][1] = 95.3;
    g_radio_state.preset_freq[RADIO_BAND_FM][2] = 101.1;
    g_radio_state.preset_freq[RADIO_BAND_FM][3] = 105.7;
    g_radio_state.preset_freq[RADIO_BAND_FM][4] = 107.9;
    g_radio_state.preset_freq[RADIO_BAND_AM][0] = 540.0;
    g_radio_state.preset_freq[RADIO_BAND_AM][1] = 720.0;
    g_radio_state.preset_freq[RADIO_BAND_AM][2] = 950.0;
    g_radio_state.preset_freq[RADIO_BAND_AM][3] = 1200.0;
    g_radio_state.preset_freq[RADIO_BAND_AM][4] = 1450.0;
    
    g_a2dp_state.volume = 10;
    g_a2dp_state.playing = false;
    
    g_hfp_state.speaker_volume = 12;
    g_hfp_state.mic_volume = 10;
    g_hfp_state.call_active = false;
    
    // Load from NVS
    load_from_nvs();
    load_bt_device_settings();
    
    // Initialize Bluetooth volume control
    bt_volume_config_t vol_config = {
        .default_a2dp_volume = g_a2dp_state.volume,
        .default_hfp_speaker_volume = g_hfp_state.speaker_volume,
        .default_hfp_mic_volume = g_hfp_state.mic_volume,
        .on_volume_change = on_bt_volume_changed
    };
    bt_volume_control_init(&vol_config);

    // Restore state if was powered on
    if (g_powered_on && g_current_mode == MODE_RADIO) {
        ESP_LOGI(TAG, "Restoring radio at %.1f MHz", g_radio_state.frequency);
        send_display_notification(DISPLAY_MODE_CHANGE, "Radio", NULL, 2000, 100);
    } else if (!g_powered_on) {
        g_current_mode = MODE_OFF;
    }
    
    return ESP_OK;
}

void stereo_state_handle_button(button_event_t event)
{
    ESP_LOGI(TAG, "State machine handling button: btn=%d, type=%d, current_mode=%d",
             event.button, event.event, g_current_mode);

    // ===== POWER CONTROL - SHORT PRESS ON ROTARY BUTTON ONLY =====
    if (event.button == BTN_ROTARY && event.event == BTN_EVENT_PRESS) {
        if (g_current_mode == MODE_OFF) {
            // Power ON
            ESP_LOGI(TAG, "Power ON");
            g_powered_on = true;
            g_current_mode = MODE_RADIO;
            send_display_notification(DISPLAY_MODE_CHANGE, "Power ON", NULL, 1500, 150);
            save_to_nvs();
            if (g_config.on_mode_change) {
                g_config.on_mode_change(MODE_OFF, g_current_mode);
            }
            return;
        } else {
            // Power OFF
            ESP_LOGI(TAG, "Power OFF");
            stereo_mode_t old_mode = g_current_mode;
            g_powered_on = false;
            g_current_mode = MODE_OFF;
            send_display_notification(DISPLAY_MODE_CHANGE, "Power OFF", NULL, 1000, 150);
            save_to_nvs();
            if (g_config.on_mode_change) {
                g_config.on_mode_change(old_mode, MODE_OFF);
            }
            return;
        }
    }

    // ===== VOICE RECOGNITION - BUTTON 1 STARTS, BUTTON 2 STOPS =====
    // BTN_BAND_UM = Button 1 (100kΩ resistor, threshold ~201)
    // BTN_BAND_VF = Button 2 (68kΩ resistor, threshold ~346)
    
    if (event.button == BTN_BAND_UM && event.event == BTN_EVENT_PRESS) {
        // Button 1: START voice recognition
        if (!g_voice_recognition_active && 
            (g_current_mode == MODE_BLUETOOTH || g_current_mode == MODE_RADIO)) {
            ESP_LOGI(TAG, "Button 1 (BAND_UM): Starting voice recognition");
            esp_err_t ret = a2dpSinkHfpHf_start_voice_recognition();
            if (ret == ESP_OK) {
                g_voice_recognition_active = true;
                send_display_notification(DISPLAY_MODE_CHANGE, 
                                        "Voice Assistant", 
                                        "Listening...", 
                                        0, 200);
            } else {
                ESP_LOGE(TAG, "Failed to start voice recognition: %s", 
                        esp_err_to_name(ret));
                send_display_notification(DISPLAY_MODE_CHANGE, 
                                        "Voice Assistant", 
                                        "Failed to Start", 
                                        2000, 200);
            }
        } else if (g_voice_recognition_active) {
            ESP_LOGW(TAG, "Voice recognition already active");
        } else {
            ESP_LOGW(TAG, "Voice recognition not available in current mode");
        }
        return;
    }

    if (event.button == BTN_BAND_VF && event.event == BTN_EVENT_PRESS) {
        // Button 2: STOP voice recognition
        if (g_voice_recognition_active) {
            ESP_LOGI(TAG, "Button 2 (BAND_VF): Stopping voice recognition");
            esp_err_t ret = a2dpSinkHfpHf_stop_voice_recognition();
            if (ret == ESP_OK) {
                g_voice_recognition_active = false;
                send_display_notification(DISPLAY_MODE_CHANGE, 
                                        "Voice Assistant", 
                                        "Stopped", 
                                        1500, 180);
            } else {
                ESP_LOGE(TAG, "Failed to stop voice recognition: %s", 
                        esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "Voice recognition not active");
        }
        return;
    }

    // If system is off, ignore all other buttons/events
    if (g_current_mode == MODE_OFF) {
        return;
    }

    // ===== MODE-SPECIFIC BUTTON HANDLING =====
    switch (g_current_mode) {
        case MODE_BLUETOOTH:
            handle_bluetooth_mode(event);
            break;
        case MODE_RADIO:
            handle_radio_mode(event);
            break;
        case MODE_PHONE_CALL:
            handle_phone_call_mode(event);
            break;
        case MODE_PHONEBOOK:
            handle_phonebook_mode(event);
            break;
        default:
            break;
    }
}

stereo_mode_t stereo_state_get_mode(void)
{
    return g_current_mode;
}

void stereo_state_set_power(bool on)
{
    if (on == g_powered_on) return;
    
    g_powered_on = on;
    
    if (on) {
        g_current_mode = MODE_RADIO;
        send_display_notification(DISPLAY_MODE_CHANGE, "Power ON", "Welcome", 2000, 200);
    } else {
        g_current_mode = MODE_OFF;
        send_display_notification(DISPLAY_MODE_CHANGE, "Power OFF", "Goodbye", 2000, 200);
    }
    
    save_to_nvs();
    if (g_config.on_mode_change) g_config.on_mode_change(MODE_OFF, g_current_mode);
}

bool stereo_state_is_powered_on(void)
{
    return g_powered_on;
}

/**
 * @brief Auto-power-on the system when triggered by BT events (only if currently OFF)
 */
static void auto_power_on_if_off(stereo_mode_t target_mode, const char *reason)
{
    if (g_powered_on) {
        // Already on - just switch mode if needed
        if (g_current_mode != target_mode && g_current_mode != MODE_PHONE_CALL) {
            stereo_mode_t old_mode = g_current_mode;
            g_current_mode = target_mode;
            send_display_notification(DISPLAY_MODE_CHANGE, 
                                     target_mode == MODE_BLUETOOTH ? "Bluetooth" : "Phone Call",
                                     reason, 1500, 180);
            save_to_nvs();
            if (g_config.on_mode_change) {
                g_config.on_mode_change(old_mode, target_mode);
            }
        }
        return;
    }
    
    // System is OFF - auto-power-on
    ESP_LOGI(TAG, "Auto-powering ON: %s", reason);
    g_powered_on = true;
    g_current_mode = target_mode;
    
    send_display_notification(DISPLAY_MODE_CHANGE, 
                             "Auto Power ON", 
                             reason, 
                             2000, 200);
    
    save_to_nvs();
    
    if (g_config.on_mode_change) {
        g_config.on_mode_change(MODE_OFF, g_current_mode);
    }
}

void stereo_state_hfp_call_status(bool call_active, const char *caller_id)
{
    if (call_active && g_current_mode != MODE_PHONE_CALL) {
        ESP_LOGI(TAG, "Incoming call: %s", caller_id ? caller_id : "Unknown");
        
        // Store caller ID
        if (caller_id) {
            strncpy(g_caller_id, caller_id, sizeof(g_caller_id) - 1);
        } else {
            strcpy(g_caller_id, "Unknown Caller");
        }
        
        // **AUTO-POWER-ON if OFF, or save previous mode if already on**
        if (!g_powered_on) {
            auto_power_on_if_off(MODE_PHONE_CALL, "Incoming Call");
        } else {
            // System already on - save previous mode
            if (g_current_mode == MODE_PHONEBOOK) {
                g_mode_before_call = g_mode_before_phonebook;
            } else {
                g_mode_before_call = g_current_mode;
            }
            g_current_mode = MODE_PHONE_CALL;
            
            if (g_config.on_mode_change) {
                g_config.on_mode_change(g_mode_before_call, MODE_PHONE_CALL);
            }
        }
        
        send_display_notification(DISPLAY_CALL_INCOMING, g_caller_id, "Press to answer", 0, 255);
        
        // Auto-answer call
        a2dpSinkHfpHf_answer_call();
        send_display_notification(DISPLAY_CALL_ACTIVE, g_caller_id, "Connected", 0, 250);
        
    } else if (!call_active && g_current_mode == MODE_PHONE_CALL) {
        ESP_LOGI(TAG, "Call ended");
        
        send_display_notification(DISPLAY_CALL_ACTIVE, "Call Ended", NULL, 2000, 200);
        
        g_current_mode = g_mode_before_call;
        
        if (g_config.on_mode_change) {
            g_config.on_mode_change(MODE_PHONE_CALL, g_current_mode);
        }
        
        save_to_nvs();
    }
}

void stereo_state_rds_update(const char *station_name, const char *song_info)
{
    if (g_current_mode != MODE_RADIO) return;
    
    if (station_name) {
        strncpy(g_radio_state.station_name, station_name, sizeof(g_radio_state.station_name) - 1);
        send_display_notification(DISPLAY_RADIO_STATION, station_name, NULL, 5000, 80);
    }
    
    if (song_info) {
        strncpy(g_radio_state.song_info, song_info, sizeof(g_radio_state.song_info) - 1);
        send_display_notification(DISPLAY_RADIO_SONG, song_info, station_name, 5000, 80);
    }
}

void stereo_state_a2dp_metadata(const char *title, const char *artist, const char *album)
{
    if (g_current_mode != MODE_BLUETOOTH) return;
    
    // Only show title/artist, not album!
    if (title) {
        strncpy(g_a2dp_state.track, title, sizeof(g_a2dp_state.track) - 1);
        g_a2dp_state.track[sizeof(g_a2dp_state.track) - 1] = '\0';
    }
    if (artist) {
        strncpy(g_a2dp_state.artist, artist, sizeof(g_a2dp_state.artist) - 1);
        g_a2dp_state.artist[sizeof(g_a2dp_state.artist) - 1] = '\0';
    }

    // Only display title on line 1, artist on line 2 for 5 seconds
    send_display_notification(
        DISPLAY_BT_TRACK,      // or an appropriate type, e.g., DISPLAY_MODE_CHANGE
        (title && *title) ? title : "",
        (artist && *artist) ? artist : "",
        5000,
        80
    );
}

void stereo_state_bt_device_connected(const uint8_t *device_addr)
{
    if (!device_addr) return;
    
    ESP_LOGI(TAG, "BT device connected: %02X:%02X:%02X:%02X:%02X:%02X",
             device_addr[0], device_addr[1], device_addr[2],
             device_addr[3], device_addr[4], device_addr[5]);
    
    memcpy(g_current_bt_device_mac, device_addr, 6);
    
    int idx = find_bt_device_by_mac(device_addr);
    if (idx >= 0) {
        // Retrieve saved volumes
        uint8_t saved_a2dp_vol = g_bt_devices[idx].a2dp_volume;
        uint8_t saved_hfp_speaker_vol = g_bt_devices[idx].hfp_speaker_volume;
        uint8_t saved_hfp_mic_vol = g_bt_devices[idx].hfp_mic_volume;
        
        ESP_LOGI(TAG, "Restoring volumes: A2DP=%d, HFP_SPK=%d, HFP_MIC=%d",
                 saved_a2dp_vol, saved_hfp_speaker_vol, saved_hfp_mic_vol);
        
        // Update global state
        g_a2dp_state.volume = saved_a2dp_vol;
        g_hfp_state.speaker_volume = saved_hfp_speaker_vol;
        g_hfp_state.mic_volume = saved_hfp_mic_vol;
        
        // Apply volumes by calling component APIs
        a2dpSinkHfpHf_set_a2dp_volume(saved_a2dp_vol);
        a2dpSinkHfpHf_set_hfp_speaker_volume(saved_hfp_speaker_vol);
        a2dpSinkHfpHf_set_hfp_mic_volume(saved_hfp_mic_vol);
        
        // Optional: Notify user
        send_display_notification(DISPLAY_MODE_CHANGE,
                                  "Device Connected",
                                  "Volumes Restored",
                                  2000, 140);
        
    } else {
        // New device - apply default volumes
        ESP_LOGI(TAG, "New device. Applying default volumes.");
        
        // Use defaults (replace with your defaults)
        uint8_t default_a2dp_volume = g_a2dp_state.volume;
        uint8_t default_hfp_speaker_volume = g_hfp_state.speaker_volume;
        uint8_t default_hfp_mic_volume = g_hfp_state.mic_volume;
        
        a2dpSinkHfpHf_set_a2dp_volume(default_a2dp_volume);
        a2dpSinkHfpHf_set_hfp_speaker_volume(default_hfp_speaker_volume);
        a2dpSinkHfpHf_set_hfp_mic_volume(default_hfp_mic_volume);
        
        send_display_notification(DISPLAY_MODE_CHANGE,
                                  "New Device",
                                  "Default Volumes",
                                  2000, 140);
    }
}


void stereo_state_bt_device_disconnected(const uint8_t *device_addr)
{
    if (!device_addr) return;
    
    ESP_LOGI(TAG, "BT device disconnected");
    
    int idx = find_bt_device_by_mac(device_addr);
    if (idx < 0) idx = find_free_bt_device_slot();
    
    memcpy(g_bt_devices[idx].mac_addr, device_addr, 6);
    g_bt_devices[idx].a2dp_volume = g_a2dp_state.volume;
    g_bt_devices[idx].hfp_speaker_volume = g_hfp_state.speaker_volume;
    g_bt_devices[idx].hfp_mic_volume = g_hfp_state.mic_volume;
    g_bt_devices[idx].valid = true;
    
    save_bt_device_settings();
    memset(g_current_bt_device_mac, 0, 6);
}

void stereo_state_save(void)
{
    save_to_nvs();
}

void stereo_state_set_mode(stereo_mode_t mode)
{
    if (mode == g_current_mode) return;
    
    stereo_mode_t old_mode = g_current_mode;
    g_current_mode = mode;
    
    const char *mode_names[] = {"OFF", "RADIO", "BLUETOOTH", "PHONE_CALL", "PHONEBOOK"};
    ESP_LOGI(TAG, "Mode changed: %s -> %s", mode_names[old_mode], mode_names[mode]);
    
    // Send display notification
    send_display_notification(DISPLAY_MODE_CHANGE, mode_names[mode], NULL, 2000, 150);
    
    // Save to NVS
    save_to_nvs();
    
    // Notify callback
    if (g_config.on_mode_change) {
        g_config.on_mode_change(old_mode, mode);
    }
}

void stereo_state_a2dp_streaming(bool streaming)
{
    g_a2dp_state.playing = streaming;
    
    if (streaming) {
        ESP_LOGI(TAG, "A2DP audio streaming started");
        // **AUTO-POWER-ON if system is OFF, or switch to BT mode if already on**
        auto_power_on_if_off(MODE_BLUETOOTH, "Music Playing");
    } else {
        ESP_LOGI(TAG, "A2DP audio streaming stopped");
    }
}
