#include "buttons.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define TAG "BUTTONS"

// Button ADC thresholds for ESP32 (12-bit, 3.3V)
// Based on measured voltages with pull-down resistor
static const uint16_t button_thresholds[] = {
    201,   // BTN_BAND_UM (Tactile 1: 100kΩ) - MEASURED ✓
    346,   // BTN_BAND_VF (Tactile 2: 68kΩ)  - MEASURED ✓
    757,   // BTN_STATION_1 (Tactile 3: 33kΩ) - MEASURED ✓
    1425,  // BTN_STATION_2 (Tactile 4: 15kΩ) - MEASURED ✓
    2204,  // BTN_STATION_3 (Tactile 5: 6.8kΩ) - MEASURED ✓
    2830,  // BTN_STATION_4 (Tactile 6: 3.3kΩ) - MEASURED ✓
    3450,  // BTN_STATION_5 (Tactile 7: 1.5kΩ) - MEASURED ✓
    3920,  // BTN_DOWN (Tactile 8: 330Ω) - MEASURED ✓
    4095   // BTN_UP (Tactile 9: 150Ω) - estimated (probably ~4050-4070)
};

#define THRESHOLD_TOLERANCE 40  // ±40 ADC counts
#define NUM_BUTTONS 9
#define ADC_SAMPLES 4  // Average 4 readings for stability

// Rotary encoder state machine
typedef enum {
    ENC_STATE_00 = 0,  // Both LOW
    ENC_STATE_01 = 1,  // DT HIGH, CLK LOW
    ENC_STATE_10 = 2,  // CLK HIGH, DT LOW
    ENC_STATE_11 = 3   // Both HIGH
} encoder_state_t;

// Global variables
static adc_oneshot_unit_handle_t adc_handle;
static adc_channel_t adc_channel;
static int g_rotary_clk_pin;
static int g_rotary_dt_pin;
static int g_rotary_sw_pin;
static button_callback_t g_callback;
static QueueHandle_t g_button_queue = NULL;
// Timing variables
static volatile uint32_t g_last_rotary_time = 0;
static volatile uint32_t g_last_sw_time = 0;
static volatile uint32_t g_rotary_press_start = 0;
static volatile bool g_rotary_pressed = false;
static volatile bool g_long_press_sent = false;

// Rotary encoder state
static volatile encoder_state_t g_encoder_state = ENC_STATE_11;  // Start at rest position
static volatile int8_t g_encoder_position = 0;

// Debounce and timing constants
#define DEBOUNCE_MS 50
#define ROTARY_DEBOUNCE_MS 5  // Faster for rotary encoder
#define ROTATION_FILTER_MS 300  // Ignore button press within 200ms of rotation
#define LONG_PRESS_THRESHOLD_MS 1000  // 1 second to trigger voice recognition

// Quadrature state machine transition table
// 0=invalid, 1=CW, -1=CCW, 0=no_move
static const int8_t rotary_transition_table[4][4] = {
    {0, -1, 1, 0},    // 00
    {1, 0, 0, -1},    // 01
    {-1, 0, 0, 1},    // 10
    {0, 1, -1, 0}     // 11
};

// Helper function: Send button event to queue
static inline void send_button_event(uint8_t button, uint8_t type)
{
    button_event_t event = {
        .button = button,
        .event = type
    };
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(g_button_queue, &event, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Rotary encoder ISR - handles both CLK and DT state changes
 * Uses proper quadrature decoding with Gray code state machine
 */
static void IRAM_ATTR rotary_encoder_isr(void *arg)
{
    uint32_t now = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    
    // Debounce - encoders typically bounce for 1-5ms
    if (now - g_last_rotary_time < ROTARY_DEBOUNCE_MS) {
        return;
    }
    
    // Read both encoder pins
    int sw_state = gpio_get_level(g_rotary_sw_pin);
    int clk_state = gpio_get_level(g_rotary_clk_pin);
    int dt_state = gpio_get_level(g_rotary_dt_pin);
    // esp_rom_printf("sw: %d, clk: %d, dt: %d\n", sw_state, clk_state, dt_state);
    // Rotary button pressed (only if rotary at rest - both CLK and DT HIGH)
    if (sw_state == 0 && clk_state == 1 && dt_state == 1) {
        if (!g_rotary_pressed) {
            g_rotary_press_start = now;
            g_rotary_pressed = true;
            g_long_press_sent = false;
            g_encoder_position = 0;
        }
        // Don't send event immediately; handled via separate monitor task for long press
    }
    // Rotary button released
    else if (sw_state == 1 && g_rotary_pressed && clk_state == 1 && dt_state == 1) {
        g_rotary_pressed = false;
        uint32_t press_duration = now - g_rotary_press_start;
        if (press_duration >= LONG_PRESS_THRESHOLD_MS) {
            send_button_event(BTN_ROTARY, BTN_EVENT_RELEASE_AFTER_LONG);
        } else if (press_duration > DEBOUNCE_MS) {
            send_button_event(BTN_ROTARY, BTN_EVENT_PRESS);
        }
    }
    // Rotary turn decoding on clk/dt edges
    else {
        // Create new state from pin readings
        encoder_state_t new_state = (encoder_state_t)((clk_state << 1) | dt_state);
        // Look up transition direction
        int8_t direction = rotary_transition_table[g_encoder_state][new_state];
        // Update state
        g_encoder_state = new_state;

        if (direction != 0) {
            // Valid rotation detected
            g_encoder_position += direction;
            g_last_rotary_time = now; // Mark last rotary activity
            // Each detent typically generates 4 state changes
            // Send event only on full detent (every 4 steps)
            // We have set it to 2 / -2 now (response every click on the dial)
            if (g_encoder_position >= 2) {
                g_encoder_position = 0;
                send_button_event(BTN_ROTARY, BTN_EVENT_ROTARY_CW);
            } else if (g_encoder_position <= -2) {
                g_encoder_position = 0;
                send_button_event(BTN_ROTARY, BTN_EVENT_ROTARY_CCW);
            }
        }
    }
}

// Read button from ADC with averaging
static button_id_t adc_read_button(void)
{
    int adc_sum = 0;
    
    // Average multiple readings
    for (int i = 0; i < ADC_SAMPLES; i++) {
        int reading;
        if (adc_oneshot_read(adc_handle, adc_channel, &reading) == ESP_OK) {
            adc_sum += reading;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    int adc_reading = adc_sum / ADC_SAMPLES;
    
    // No button pressed
    if (adc_reading < 100) {
        return BTN_NONE;
    }
    

return BTN_NONE;

    // Find matching button
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (adc_reading >= (button_thresholds[i] - THRESHOLD_TOLERANCE) &&
            adc_reading <= (button_thresholds[i] + THRESHOLD_TOLERANCE)) {
            // ESP_LOGI(TAG, "Button %d matched! (threshold: %d, reading: %d)", 
            //          i+1, button_thresholds[i], adc_reading);
            return (button_id_t)(i + 1);  // +1 because BTN_ROTARY is 0
        }
    }
    
    ESP_LOGW(TAG, "Unknown ADC value: %d (no button matched)", adc_reading);
    return BTN_NONE;
}

// ADC button monitoring task
static void adc_button_monitor_task(void *arg)
{
    button_id_t last_button = BTN_NONE;
    uint32_t press_time = 0;
    bool long_press_sent = false;
    uint32_t last_repeat_time = 0;  // ✅ Track last repeat event time
    
    #define BUTTON_REPEAT_INTERVAL_MS 200  // ✅ Only send repeat every 200ms
    
    while (1) {
        button_id_t current_button = adc_read_button();
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if (current_button != BTN_NONE && current_button != last_button) {
            // New button press
            press_time = now;
            last_repeat_time = now;  // ✅ Initialize repeat timer
            long_press_sent = false;
            
            button_event_t event = {
                .button = current_button,
                .event = BTN_EVENT_PRESS
            };
            
            if (g_callback) {
                g_callback(event);
            }
            
        } else if (current_button != BTN_NONE && current_button == last_button) {
            // Button held
            
            // Check for long press (only send once)
            if (!long_press_sent && (now - press_time) >= 1000) {
                button_event_t event = {
                    .button = current_button,
                    .event = BTN_EVENT_LONG_PRESS
                };
                
                if (g_callback) {
                    g_callback(event);
                }
                
                long_press_sent = true;
            }
            
            // ✅ Send repeat events but rate-limited
            if (long_press_sent && (now - last_repeat_time) >= BUTTON_REPEAT_INTERVAL_MS) {
                button_event_t event = {
                    .button = current_button,
                    .event = BTN_EVENT_REPEAT  // You need to add this to your enum
                };
                
                if (g_callback) {
                    g_callback(event);
                }
                
                last_repeat_time = now;
            }
            
        } else if (current_button == BTN_NONE && last_button != BTN_NONE) {
            // Button released
            button_event_t event = {
                .button = last_button,
                .event = BTN_EVENT_RELEASE
            };
            
            if (g_callback) {
                g_callback(event);
            }
        }
        
        last_button = current_button;
        vTaskDelay(pdMS_TO_TICKS(20)); // Check every 20ms
    }
}

/**
 * @brief Task to monitor rotary button long press
 */
static void button_long_press_monitor_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
        
        if (g_rotary_pressed && !g_long_press_sent) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            uint32_t press_duration = now - g_rotary_press_start;
            
            if (press_duration >= LONG_PRESS_THRESHOLD_MS) {
                // Send long press event ONCE
                button_event_t event = {
                    .button = BTN_ROTARY,
                    .event = BTN_EVENT_LONG_PRESS
                };
                
                if (g_callback) {
                    g_callback(event);
                }
                
                g_long_press_sent = true;  // Prevents repeated calls
            }
        }
    }
}

/**
 * @brief Rotary encoder event processing task
 */
static void rotary_event_task(void *arg)
{
    button_event_t event;
    while (1) {
        if (xQueueReceive(g_button_queue, &event, portMAX_DELAY)) {
            if (g_callback) {
                g_callback(event);
            }
        }
    }
}

/**
 * @brief Initialize button system
 */
esp_err_t buttons_init(int adc_pin, int rotary_clk, int rotary_dt, int rotary_sw, button_callback_t callback)
{
    esp_err_t ret;
    
    g_rotary_clk_pin = rotary_clk;
    g_rotary_dt_pin = rotary_dt;
    g_rotary_sw_pin = rotary_sw;
    g_callback = callback;
    
    // Create event queue
    g_button_queue = xQueueCreate(20, sizeof(button_event_t));  // Larger queue for encoder
    if (!g_button_queue) {
        ESP_LOGE(TAG, "Failed to create button queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure ADC channel
    adc_channel = ADC_CHANNEL_6;  // GPIO34 = ADC1_CH6
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12  // 0-3.3V range
    };
    ret = adc_oneshot_config_channel(adc_handle, adc_channel, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
     // Configure rotary encoder pins - IMPORTANT: Use interrupt on BOTH edges
    gpio_config_t encoder_conf = {
        .pin_bit_mask = (1ULL << rotary_clk) | (1ULL << rotary_dt),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE  // Trigger on BOTH edges for quadrature
    };
    gpio_config(&encoder_conf);
    
    // Configure rotary switch button pin
    gpio_config_t sw_conf = {
        .pin_bit_mask = (1ULL << rotary_sw),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // Enable pull-up if no external resistor
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE  // Detect both press AND release
    };
    gpio_config(&sw_conf);
    
    // Install GPIO ISR service
    gpio_install_isr_service(0);
    
    // Attach ISRs - BOTH encoder pins use same ISR
    gpio_isr_handler_add(rotary_clk, rotary_encoder_isr, NULL);
    gpio_isr_handler_add(rotary_dt, rotary_encoder_isr, NULL);
    gpio_isr_handler_add(rotary_sw, rotary_encoder_isr, NULL);
    
    // Start button monitoring task
    xTaskCreate(adc_button_monitor_task, "button_monitor", 4096, NULL, 5, NULL);
    
    // Start rotary event processing task
    xTaskCreate(rotary_event_task, "rotary_event", 4096, NULL, 5, NULL);
    
    // Start rotary push long press task
    xTaskCreate(button_long_press_monitor_task, "btn_monitor", 3072, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Button system initialized");
    ESP_LOGI(TAG, "  ADC pin: GPIO%d, Rotary: CLK=%d, DT=%d, SW=%d", 
             adc_pin, rotary_clk, rotary_dt, rotary_sw);
    
    return ESP_OK;
}