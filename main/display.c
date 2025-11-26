#include "display.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdint.h>

#define TAG "DISPLAY"

// PCF8574 I2C LCD adapter pin mapping
#define LCD_BIT_RS    (1 << 0)
#define LCD_BIT_RW    (1 << 1)
#define LCD_BIT_E     (1 << 2)
#define LCD_BIT_BL    (1 << 3)
#define LCD_BIT_D4    (1 << 4)
#define LCD_BIT_D5    (1 << 5)
#define LCD_BIT_D6    (1 << 6)
#define LCD_BIT_D7    (1 << 7)

// HD44780 commands
#define LCD_CMD_CLEAR           0x01
#define LCD_CMD_HOME            0x02
#define LCD_CMD_ENTRY_MODE      0x04
#define LCD_CMD_DISPLAY_CTRL    0x08
#define LCD_CMD_SHIFT           0x10
#define LCD_CMD_FUNCTION        0x20
#define LCD_CMD_CGRAM_ADDR      0x40
#define LCD_CMD_DDRAM_ADDR      0x80

// Entry mode flags
#define LCD_ENTRY_INC           0x02
#define LCD_ENTRY_SHIFT         0x01

// Display control flags
#define LCD_DISPLAY_ON          0x04
#define LCD_CURSOR_ON           0x02
#define LCD_BLINK_ON            0x01

// Function set flags
#define LCD_8BIT_MODE           0x10
#define LCD_4BIT_MODE           0x00
#define LCD_2_LINE              0x08
#define LCD_1_LINE              0x00
#define LCD_5x10_DOTS           0x04
#define LCD_5x8_DOTS            0x00

#define DEBUG_DISPLAY 0  // Set to 0 to disable debug; 1 to enable

static bool display_initialized = false;
static uint8_t backlight_state = LCD_BIT_BL;




// Write byte to PCF8574
static esp_err_t pcf8574_write(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(LCD_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Pulse enable pin
static void lcd_strobe_enable(uint8_t data)
{
    pcf8574_write(data | LCD_BIT_E);
    esp_rom_delay_us(1);
    pcf8574_write(data & ~LCD_BIT_E);
    esp_rom_delay_us(50);
}

// Write 4 bits to LCD
static void lcd_write_4bits(uint8_t data, bool is_data)
{
    uint8_t upper = (data & 0xF0);
    uint8_t output = upper | backlight_state;
    
    if (is_data) {
        output |= LCD_BIT_RS;
    }
    
    pcf8574_write(output);
    lcd_strobe_enable(output);
}

// Write byte in 4-bit mode
static void lcd_write_byte(uint8_t data, bool is_data)
{
    lcd_write_4bits(data & 0xF0, is_data);
    lcd_write_4bits((data << 4) & 0xF0, is_data);
    esp_rom_delay_us(50);
}

// Update lcd_command function:
static void lcd_command(uint8_t cmd)
{
    #if DEBUG_DISPLAY
    ESP_LOGI(TAG, "CMD: 0x%02X", cmd);
    #endif
    
    lcd_write_byte(cmd, false);
    if (cmd == LCD_CMD_CLEAR || cmd == LCD_CMD_HOME) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// Update lcd_data function:
static void lcd_data(uint8_t data)
{
    #if DEBUG_DISPLAY
    ESP_LOGI(TAG, "DATA: '%c' (0x%02X)", (data >= 32 && data <= 126) ? data : '.', data);
    #endif
    
    lcd_write_byte(data, true);
}

// Initialize LCD
esp_err_t display_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C LCD1602 (SLC1602A3)");
    
    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = LCD_I2C_SDA,
        .scl_io_num = LCD_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LCD_I2C_FREQ,
    };
    
    esp_err_t ret = i2c_param_config(LCD_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(LCD_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C OK - SDA:%d SCL:%d addr:0x%02X", 
             LCD_I2C_SDA, LCD_I2C_SCL, LCD_I2C_ADDR);
    
    // Wait for LCD power-up
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Initialize LCD in 4-bit mode per HD44780 datasheet
    // Start in 8-bit mode
    lcd_write_4bits(0x30, false);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    lcd_write_4bits(0x30, false);
    esp_rom_delay_us(50);
    
    lcd_write_4bits(0x30, false);
    esp_rom_delay_us(50);
    
    // Switch to 4-bit mode
    lcd_write_4bits(0x20, false);
    esp_rom_delay_us(50);
    
    // Configure display
    lcd_command(LCD_CMD_FUNCTION | LCD_4BIT_MODE | LCD_2_LINE | LCD_5x8_DOTS);
    lcd_command(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON);
    lcd_command(LCD_CMD_CLEAR);
    lcd_command(LCD_CMD_ENTRY_MODE | LCD_ENTRY_INC);
    
    display_initialized = true;
    ESP_LOGI(TAG, "LCD initialized successfully!");
    
    display_show_splash();
    return ESP_OK;
}

// Clear display
void display_clear(void)
{
    if (!display_initialized) return;
    lcd_command(LCD_CMD_CLEAR);
}

// Set cursor position
static void lcd_set_cursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= DISPLAY_ROWS) row = 0;
    if (col >= DISPLAY_COLS) col = 0;
    lcd_command(LCD_CMD_DDRAM_ADDR | (col + row_offsets[row]));
}

// Print string
static void lcd_print(const char *str)
{
    while (*str) {
        lcd_data((uint8_t)*str++);
    }
}

// Show splash screen
void display_show_splash(void)
{
    if (!display_initialized) return;
    
    ESP_LOGI(TAG, "Displaying splash screen...");
    display_clear();
    
    lcd_set_cursor(1, 0);
    lcd_print("Car Stereo");
    
    lcd_set_cursor(0, 1);
    lcd_print("ESP32 Audio");
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    display_clear();
}

// Update display
void display_update(const display_state_t *state)
{
    if (!display_initialized || !state) return;
    
    display_clear();
    
    // Line 1
    lcd_set_cursor(0, 0);
    if (strlen(state->line1) > 0) {
        char line[17];
        strncpy(line, state->line1, 16);
        line[16] = '\0';
        lcd_print(line);
    } else {
        switch(state->mode) {
            case DISPLAY_MODE_RADIO:
                lcd_print("FM Radio");
                break;
            case DISPLAY_MODE_BLUETOOTH:
                lcd_print(state->connected ? "BT: Connected" : "BT: Waiting");
                break;
            case DISPLAY_MODE_PHONE_CALL:
                lcd_print("CALL");
                break;
            case DISPLAY_MODE_OFF:
                lcd_print("System OFF");
                break;
            default:
                break;
        }
    }
    
    // Line 2
    lcd_set_cursor(0, 1);
    if (strlen(state->line2) > 0) {
        char line[17];
        strncpy(line, state->line2, 16);
        line[16] = '\0';
        lcd_print(line);
    } else {
        char status[17];
        snprintf(status, sizeof(status), "Vol:%02d %s", 
                state->volume, 
                state->playing ? ">" : " ");
        lcd_print(status);
    }
}

// Show notification
void display_notification(const char *line1, const char *line2, uint16_t duration_ms)
{
    if (!display_initialized) return;
    
    display_clear();
    
    if (line1) {
        lcd_set_cursor(0, 0);
        char line[17];
        strncpy(line, line1, 16);
        line[16] = '\0';
        lcd_print(line);
    }
    
    if (line2) {
        lcd_set_cursor(0, 1);
        char line[17];
        strncpy(line, line2, 16);
        line[16] = '\0';
        lcd_print(line);
    }
    
    if (duration_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }
}

// Backlight control
void display_set_backlight(bool on)
{
    backlight_state = on ? LCD_BIT_BL : 0;
    pcf8574_write(backlight_state);
}



// Add this test function
void display_test_simple(void)
{
    ESP_LOGI(TAG, "=== SIMPLE DISPLAY TEST ===");
    
    // Test 1: Clear and write 'A'
    ESP_LOGI(TAG, "Test 1: Clear display");
    lcd_command(LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Test 2: Write 'A' to position 0,0");
    lcd_set_cursor(0, 0);
    lcd_data('A');
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "Test 3: Write 'HELLO'");
    lcd_set_cursor(0, 0);
    lcd_data('H');
    lcd_data('E');
    lcd_data('L');
    lcd_data('L');
    lcd_data('O');
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "Test 4: Full screen test");
    lcd_command(LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Fill line 1
    lcd_set_cursor(0, 0);
    for (char c = '0'; c <= '9'; c++) {
        lcd_data(c);
    }
    for (char c = 'A'; c <= 'F'; c++) {
        lcd_data(c);
    }
    
    // Fill line 2
    lcd_set_cursor(0, 1);
    for (char c = 'a'; c <= 'p'; c++) {
        lcd_data(c);
    }
    
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    ESP_LOGI(TAG, "=== TEST COMPLETE ===");
    ESP_LOGI(TAG, "Did you see any characters on the display?");
}

// Handler for state machine display notifications
void display_handle_notification(display_notification_t notification)
{
    if (!display_initialized) return;
    
    display_state_t state = {0};
    
    // Get current mode
    stereo_mode_t current_mode = stereo_state_get_mode();
    
    switch(current_mode) {
        case MODE_OFF:
            state.mode = DISPLAY_MODE_OFF;
            display_clear();
            lcd_set_cursor(3, 0);
            lcd_print("System OFF");
            return;
            
        case MODE_RADIO:
            state.mode = DISPLAY_MODE_RADIO;
            state.volume = 10;  // Default, will be overridden by notification
            
            // Use notification data
            if (notification.text[0] != '\0') {
                strncpy(state.line1, notification.text, 16);
                state.line1[16] = '\0';
            }
            if (notification.subtext[0] != '\0') {
                strncpy(state.line2, notification.subtext, 16);
                state.line2[16] = '\0';
            }
            break;
            
        case MODE_BLUETOOTH:
            state.mode = DISPLAY_MODE_BLUETOOTH;
            state.connected = true;
            state.playing = true;
            state.volume = 10;  // Default
            
            // Show track/artist from notification
            if (notification.text[0] != '\0') {
                strncpy(state.line1, notification.text, 16);
                state.line1[16] = '\0';
            } else {
                strncpy(state.line1, "BT: Connected", 16);
            }
            
            if (notification.subtext[0] != '\0') {
                strncpy(state.line2, notification.subtext, 16);
                state.line2[16] = '\0';
            }
            break;
            
        case MODE_PHONE_CALL:
            state.mode = DISPLAY_MODE_PHONE_CALL;
            state.volume = 10;  // Default
            
            // Show call info
            if (notification.text[0] != '\0') {
                strncpy(state.line1, notification.text, 16);
                state.line1[16] = '\0';
            } else {
                strncpy(state.line1, "CALL", 16);
            }
            
            if (notification.subtext[0] != '\0') {
                strncpy(state.line2, notification.subtext, 16);
                state.line2[16] = '\0';
            }
            break;
            
        case MODE_PHONEBOOK:
            state.mode = DISPLAY_MODE_PHONEBOOK;
            
            if (notification.text[0] != '\0') {
                strncpy(state.line1, notification.text, 16);
                state.line1[16] = '\0';
            } else {
                strncpy(state.line1, "Phonebook", 16);
            }
            
            if (notification.subtext[0] != '\0') {
                strncpy(state.line2, notification.subtext, 16);
                state.line2[16] = '\0';
            }
            break;
            
        default:
            return;
    }
    
    display_update(&state);
}


// PASS 1: Strip known Spotify/streaming patterns (bullet + trailing text)
static void strip_known_patterns(char *str)
{
    if (!str) return;

    // Look for UTF-8 bullet (E2 80 A2) and remove it + everything after
    char *bullet_pos = strstr(str, "\xE2\x80\xA2");  // UTF-8 bullet
    if (bullet_pos) {
        *bullet_pos = '\0';  // Truncate at bullet
    }

    // Also strip ASCII patterns like " ? " just in case
    char *pos = strstr(str, " ? ");
    if (pos) {
        *pos = '\0';
    }

    // Trim trailing spaces
    size_t len = strlen(str);
    while (len > 0 && (str[len - 1] == ' ' || str[len - 1] == '\t')) {
        str[--len] = '\0';
    }
}

// PASS 2: Convert remaining UTF-8 sequences to ASCII
static void convert_utf8_to_ascii(char *str)
{
    if (!str) return;

    char temp[256];
    size_t dst = 0, src = 0;
    size_t len = strlen(str);

    while (src < len && dst < sizeof(temp) - 1) {
        unsigned char c = (unsigned char)str[src];

        // ASCII printable - keep as-is
        if (c >= 0x20 && c <= 0x7E) {
            temp[dst++] = str[src++];
        }
        // UTF-8 3-byte (E2 80 xx - dashes, quotes, etc.)
        else if (c == 0xE2 && src + 2 < len) {
            unsigned char b2 = (unsigned char)str[src + 1];
            unsigned char b3 = (unsigned char)str[src + 2];
            
            if (b2 == 0x80) {
                if (b3 >= 0x90 && b3 <= 0x95) temp[dst++] = '-';
                else if (b3 == 0x98 || b3 == 0x99) temp[dst++] = '\'';
                else if (b3 == 0x9C || b3 == 0x9D) temp[dst++] = '"';
                else if (b3 == 0xA2) temp[dst++] = '*';
                else temp[dst++] = '?';
            } else {
                temp[dst++] = '?';
            }
            src += 3;
        }
        // UTF-8 2-byte (C3 xx - accented chars)
        else if (c == 0xC3 && src + 1 < len) {
            unsigned char next = (unsigned char)str[src + 1];
            if (next >= 0x80 && next <= 0xBF) {
                temp[dst++] = 'a' + ((next - 0x80) % 32);
            }
            src += 2;
        }
        // Skip other non-ASCII
        else {
            src++;
        }
    }

    temp[dst] = '\0';
    strcpy(str, temp);
}

// PASS 3: Scan backward and truncate at first non-ASCII
static void truncate_trailing_garbage(char *str)
{
    if (!str) return;

    size_t len = strlen(str);
    
    while (len > 0) {
        unsigned char c = (unsigned char)str[len - 1];
        
        // Valid ASCII printable = keep
        if (c >= 0x20 && c <= 0x7E) {
            break;
        }
        
        // Garbage = truncate
        str[--len] = '\0';
    }

    // Trim trailing spaces
    while (len > 0 && (str[len - 1] == ' ' || str[len - 1] == '\t')) {
        str[--len] = '\0';
    }
}

// Combined cleanup - Strip patterns FIRST, then convert, then cleanup
void strip_spotify_junk(char *str)
{
    if (!str) return;
    
    // Pass 1: FIRST - Strip known Spotify/streaming patterns (bullet + everything after)
    strip_known_patterns(str);
    
    // Pass 2: Convert UTF-8 to ASCII (for any remaining special chars)
    convert_utf8_to_ascii(str);
    
    // Pass 3: Truncate trailing garbage (e.g., invalid bytes)
    truncate_trailing_garbage(str);
}


// Simplified sanitize_for_lcd - just removes leading control chars
void sanitize_for_lcd(char *dest, const char *src, size_t max_len)
{
    if (!src || !dest || max_len < 2) return;

    char temp[256];
    strncpy(temp, src, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    // Trim leading control/non-ASCII
    size_t start = 0;
    while (temp[start] && (unsigned char)temp[start] < 0x20) start++;
    memmove(temp, temp + start, strlen(temp + start) + 1);

    // Clean with two-pass approach
    strip_spotify_junk(temp);

    // Copy to dest (now guaranteed ASCII-clean)
    strncpy(dest, temp, max_len - 1);
    dest[max_len - 1] = '\0';
}
