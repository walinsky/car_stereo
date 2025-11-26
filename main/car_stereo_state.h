#ifndef CAR_STEREO_STATE_H
#define CAR_STEREO_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "buttons.h"

#ifdef __cplusplus
extern "C" {
#endif

// Operating modes
typedef enum {
    MODE_OFF,           // Power off state
    MODE_RADIO,         // FM Radio
    MODE_BLUETOOTH,     // A2DP Audio
    MODE_PHONE_CALL,    // Active HFP call
    MODE_PHONEBOOK      // Browsing phonebook
} stereo_mode_t;

// Display notification types
typedef enum {
    DISPLAY_RADIO_STATION,      // RDS station name
    DISPLAY_RADIO_SONG,         // RDS song/artist info
    DISPLAY_BT_TRACK,           // A2DP track title
    DISPLAY_BT_ARTIST,          // A2DP artist
    DISPLAY_BT_ALBUM,           // A2DP album
    DISPLAY_CALL_INCOMING,      // Incoming call (caller ID)
    DISPLAY_CALL_ACTIVE,        // Active call status
    DISPLAY_PHONEBOOK_CONTACT,  // Phonebook contact being browsed
    DISPLAY_VOLUME,             // Volume level
    DISPLAY_FREQUENCY,          // Radio frequency
    DISPLAY_MODE_CHANGE         // Mode changed
} display_notification_type_t;

// Display notification structure
typedef struct {
    display_notification_type_t type;
    char text[128];             // Main text to display
    char subtext[64];           // Secondary text (optional)
    uint32_t duration_ms;       // How long to display (0 = permanent)
    uint8_t priority;           // Priority (0-255, higher = more important)
} display_notification_t;

typedef enum {
    RADIO_BAND_FM = 0,
    RADIO_BAND_AM = 1,
    RADIO_BAND_COUNT = 2
} radio_band_t;

/**
 * @brief A2DP (music streaming) state
 */
typedef struct {
    uint8_t volume;           // A2DP volume (0-15)
    bool playing;             // Currently playing music
    char track[64];           // Current track title
    char artist[64];          // Current artist
    char album[64];           // Current album (if needed)
} a2dp_state_t;

/**
 * @brief HFP (hands-free) state
 */
typedef struct {
    uint8_t speaker_volume;   // HFP speaker volume (0-15)
    uint8_t mic_volume;       // HFP microphone volume (0-15)
    bool call_active;         // Currently in a call
    char caller_id[64];       // Current caller ID
} hfp_state_t;

/**
 * @brief Radio state
 */
typedef struct {
    float frequency;          // Current frequency
    uint8_t volume;           // Radio volume (0-15)
    radio_band_t band;        // Current band (FM/AM)
    float preset_freq[2][5];  // Presets [band][preset_num]
    char station_name[32];    // RDS station name
    char song_info[64];       // RDS song info
} radio_state_t;

// Phonebook browsing state
typedef struct {
    char current_letter;
    uint16_t contact_index;
    char contact_name[64];
    char phone_number[32];
} phonebook_state_t;

// Display notification callback
typedef void (*display_callback_t)(display_notification_t notification);

// Configuration
typedef struct {
    void *fm_radio_handle;      // FM radio component handle
    display_callback_t display_handler;  // Display notification handler
    void (*on_mode_change)(stereo_mode_t old_mode, stereo_mode_t new_mode);
} stereo_config_t;

/**
 * @brief Initialize car stereo state machine
 * Loads last state from NVS
 * @param config Configuration structure
 * @return ESP_OK on success
 */
esp_err_t stereo_state_init(const stereo_config_t *config);

/**
 * @brief Handle button event (called by button callback)
 * @param event Button event to process
 */
void stereo_state_handle_button(button_event_t event);

/**
 * @brief Handle Bluetooth device connection
 * Called when a device connects - loads its saved settings
 * @param device_addr Bluetooth MAC address of connected device
 */
void stereo_state_bt_device_connected(const uint8_t *device_addr);

/**
 * @brief Handle Bluetooth device disconnection
 * Called when device disconnects - saves its settings
 * @param device_addr Bluetooth MAC address of disconnected device
 */
void stereo_state_bt_device_disconnected(const uint8_t *device_addr);

/**
 * @brief Get current operating mode
 * @return Current mode
 */
stereo_mode_t stereo_state_get_mode(void);

/**
 * @brief Set power state
 * @param on True to power on, false to power off
 */
void stereo_state_set_power(bool on);

/**
 * @brief Get power state
 * @return True if powered on
 */
bool stereo_state_is_powered_on(void);

/**
 * @brief Handle HFP call state change (called by bluetooth component)
 * @param call_active True if call is active
 * @param caller_id Caller ID string (NULL if unavailable)
 */
void stereo_state_hfp_call_status(bool call_active, const char *caller_id);

/**
 * @brief Handle RDS data from FM radio
 * @param station_name Station name (or NULL)
 * @param song_info Song/artist info (or NULL)
 */
void stereo_state_rds_update(const char *station_name, const char *song_info);

/**
 * @brief Handle A2DP metadata from bluetooth
 * @param title Track title (or NULL)
 * @param artist Artist name (or NULL)
 * @param album Album name (or NULL)
 */
void stereo_state_a2dp_metadata(const char *title, const char *artist, const char *album);

/**
 * @brief Save current state to NVS (for power cycle persistence)
 * Called automatically, but can be called manually for immediate save
 */
void stereo_state_save(void);

/**
 * @brief Set operating mode
 * @param mode Mode to switch to
 */
void stereo_state_set_mode(stereo_mode_t mode);

/**
 * @brief Notify that A2DP audio streaming state changed
 * @param streaming True if streaming, false if stopped
 */
void stereo_state_a2dp_streaming(bool streaming);

#ifdef __cplusplus
}
#endif

#endif // CAR_STEREO_STATE_H
