/* =========================================================================
 * ROBOT PENYEIMBANG ESP-IDF - Versi Terstruktur & Dioptimasi
 * 
 * Perbaikan utama:
 * - Struktur kode lebih modular
 * - Penanganan error yang lebih baik
 * - Konstanta terorganisir
 * - Logging yang informatif
 * - Manajemen memori yang lebih aman
 * ========================================================================= */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "BALANCE_BOT";

/* =========================================================================
 * 1. KONFIGURASI & KONSTANTA
 * ========================================================================= */

// Pin Definitions
#define MOTOR_LEFT_IN1     26
#define MOTOR_LEFT_IN2     17
#define MOTOR_LEFT_EN      22
#define MOTOR_RIGHT_IN1    16
#define MOTOR_RIGHT_IN2    27
#define MOTOR_RIGHT_EN     25

#define MPU6050_SDA_PIN    33
#define MPU6050_SCL_PIN    32
#define BUTTON_GPIO        19
#define LED_GPIO           2

// I2C Configuration
#define MPU6050_I2C_PORT   I2C_NUM_0
#define MPU6050_ADDR       0x69

// PWM Configuration
#define LEDC_TIMER         LEDC_TIMER_0
#define LEDC_MODE          LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_LEFT  LEDC_CHANNEL_0
#define LEDC_CHANNEL_RIGHT LEDC_CHANNEL_1
#define LEDC_RESOLUTION    LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY_HZ  5000

// MPU6050 Register Map
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1   0x6B

// PID Constants (SESUAIKAN SAAT TUNING)
#define KP          90.0f
#define KI          5.0f
#define KD          0.000f
#define SETPOINT_ANGLE -4.0f

// System Constants
#define MAX_ANGLE_FALL       30.0f
#define MAX_ANGLE_RECOVER    15.0f
#define LOOP_TIME_MS         20
#define MOTOR_DEADBAND       210
#define ALPHA                0.98f
#define INTEGRAL_MAX_CLAMP   150.0f
#define ACCEL_SCALE_FACTOR   16384.0f
#define GYRO_SCALE_FACTOR    131.0f

// Calibration Constants
#define GYRO_CALIBRATION_SAMPLES  1000
#define ANGLE_INIT_SAMPLES        100

/* =========================================================================
 * 2. VARIABEL GLOBAL & STRUCTURES
 * ========================================================================= */

typedef struct {
    bool motor_active;
    bool safety_lock;
    float integral_error;
    float previous_error;
    float setpoint_angle;
    float angle_pitch;
    float angle_roll;
} system_state_t;

typedef struct {
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
} calibration_data_t;

// Global Handles
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t mpu6050_handle = NULL;
static QueueHandle_t g_button_queue = NULL;

// Global State
static system_state_t g_state = {
    .motor_active = false,
    .safety_lock = true,
    .integral_error = 0.0f,
    .previous_error = 0.0f,
    .setpoint_angle = SETPOINT_ANGLE,
    .angle_pitch = 0.0f,
    .angle_roll = 0.0f
};

static calibration_data_t g_calibration = {0};

/* =========================================================================
 * 3. DEKLARASI FUNGSI
 * ========================================================================= */

// Initialization Functions
static esp_err_t hardware_init(void);
static esp_err_t i2c_master_init_new(void);
static esp_err_t motor_io_init(void);
static esp_err_t motor_pwm_init(void);
static esp_err_t system_io_init(void);

// MPU6050 Functions
static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data);
static esp_err_t mpu6050_read_bytes(uint8_t reg_start, uint8_t *data_buf, uint8_t len);
static esp_err_t mpu6050_sensor_init(void);
static void mpu_system_init_and_calibrate(void);

// Motor Control Functions
static void motor_set_speed(int speed);
static void reset_pid(void);
static inline int16_t combine_bytes(uint8_t high, uint8_t low);

// Task Functions
static void IRAM_ATTR button_isr_handler(void* arg);
void button_task(void* arg);
void balancing_task(void* arg);

/* =========================================================================
 * 4. IMPLEMENTASI FUNGSI INITIALIZATION
 * ========================================================================= */

static esp_err_t i2c_master_init_new(void)
{
    ESP_LOGI(TAG, "Initializing I2C Master (Driver v5.x)...");
    
    i2c_master_bus_config_t i2c_bus_conf = {
        .i2c_port = MPU6050_I2C_PORT,
        .sda_io_num = MPU6050_SDA_PIN,
        .scl_io_num = MPU6050_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    
    esp_err_t ret = i2c_new_master_bus(&i2c_bus_conf, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t mpu6050_dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 400000,
    };
    
    ret = i2c_master_bus_add_device(i2c_bus_handle, &mpu6050_dev_conf, &mpu6050_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MPU6050 device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = NULL;
    }
    
    return ret;
}

static esp_err_t motor_io_init(void)
{
    ESP_LOGI(TAG, "Initializing Motor Direction Pins...");
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_LEFT_IN1) | (1ULL << MOTOR_LEFT_IN2) |
                        (1ULL << MOTOR_RIGHT_IN1) | (1ULL << MOTOR_RIGHT_IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        // Set initial motor states to brake
        gpio_set_level(MOTOR_LEFT_IN1, 1);
        gpio_set_level(MOTOR_LEFT_IN2, 1);
        gpio_set_level(MOTOR_RIGHT_IN1, 1);
        gpio_set_level(MOTOR_RIGHT_IN2, 1);
    }
    
    return ret;
}

static esp_err_t motor_pwm_init(void)
{
    ESP_LOGI(TAG, "Initializing Motor PWM...");
    
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz = LEDC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t left_channel_conf = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_LEFT,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_LEFT_EN,
        .duty = 0,
        .hpoint = 0,
    };

    ledc_channel_config_t right_channel_conf = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_RIGHT,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_RIGHT_EN,
        .duty = 0,
        .hpoint = 0,
    };

    ret = ledc_channel_config(&left_channel_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure left motor channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_channel_config(&right_channel_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure right motor channel: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

static esp_err_t system_io_init(void)
{
    ESP_LOGI(TAG, "Initializing System IO...");
    
    // LED Configuration
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&led_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LED: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(LED_GPIO, 0);

    // Button Configuration
    gpio_config_t button_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    
    ret = gpio_config(&button_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure button: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create button queue
    g_button_queue = xQueueCreate(5, sizeof(uint32_t));
    if (g_button_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create button queue");
        return ESP_FAIL;
    }

    // Install ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add button ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

static esp_err_t hardware_init(void)
{
    ESP_LOGI(TAG, "Starting hardware initialization...");
    
    esp_err_t ret = ESP_OK;
    
    if ((ret = i2c_master_init_new()) != ESP_OK) return ret;
    if ((ret = motor_io_init()) != ESP_OK) return ret;
    if ((ret = motor_pwm_init()) != ESP_OK) return ret;
    if ((ret = system_io_init()) != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "Hardware initialization completed successfully");
    return ret;
}

/* =========================================================================
 * 5. IMPLEMENTASI FUNGSI MPU6050
 * ========================================================================= */

static inline int16_t combine_bytes(uint8_t high, uint8_t low)
{
    return (int16_t)((high << 8) | low);
}

static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(mpu6050_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_read_bytes(uint8_t reg_start, uint8_t *data_buf, uint8_t len)
{
    return i2c_master_transmit_receive(mpu6050_handle, &reg_start, 1, data_buf, len, pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_sensor_init(void)
{
    ESP_LOGI(TAG, "Initializing MPU6050 sensor...");
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    esp_err_t ret;
    
    // Reset device and set clock source
    if ((ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x80)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset MPU6050");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    if ((ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x01)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set clock source");
        return ret;
    }
    
    // Configure sensor ranges
    if ((ret = mpu6050_write_byte(MPU6050_GYRO_CONFIG, 0x00)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyro");
        return ret;
    }
    
    if ((ret = mpu6050_write_byte(MPU6050_ACCEL_CONFIG, 0x00)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return ret;
    }
    
    // Configure DLPF
    if ((ret = mpu6050_write_byte(MPU6050_CONFIG, 0x03)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DLPF");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "MPU6050 initialization completed");
    return ESP_OK;
}

static void mpu_system_init_and_calibrate(void)
{
    ESP_LOGI(TAG, "Starting MPU6050 calibration...");
    
    if (mpu6050_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 initialization failed!");
        return;
    }

    // Gyroscope Calibration
    ESP_LOGI(TAG, "Calibrating gyroscope (DO NOT MOVE ROBOT)...");
    
    int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
    uint8_t gyro_data[6];
    int successful_reads = 0;
    
    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
        if (mpu6050_read_bytes(MPU6050_GYRO_XOUT_H, gyro_data, 6) == ESP_OK) {
            gyro_x_sum += combine_bytes(gyro_data[0], gyro_data[1]);
            gyro_y_sum += combine_bytes(gyro_data[2], gyro_data[3]);
            gyro_z_sum += combine_bytes(gyro_data[4], gyro_data[5]);
            successful_reads++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    if (successful_reads > 0) {
        g_calibration.gyro_x_offset = (float)gyro_x_sum / successful_reads;
        g_calibration.gyro_y_offset = (float)gyro_y_sum / successful_reads;
        g_calibration.gyro_z_offset = (float)gyro_z_sum / successful_reads;
        ESP_LOGI(TAG, "Gyro calibration completed: X=%.2f, Y=%.2f, Z=%.2f", 
                 g_calibration.gyro_x_offset, g_calibration.gyro_y_offset, g_calibration.gyro_z_offset);
    } else {
        ESP_LOGE(TAG, "Gyro calibration failed - no successful reads");
        return;
    }

    // Initial Angle Calculation
    ESP_LOGI(TAG, "Calculating initial angles...");
    
    uint8_t accel_data[6];
    float pitch_sum = 0.0f, roll_sum = 0.0f;
    successful_reads = 0;
    
    for (int i = 0; i < ANGLE_INIT_SAMPLES; i++) {
        if (mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, accel_data, 6) == ESP_OK) {
            int16_t ax = combine_bytes(accel_data[0], accel_data[1]);
            int16_t ay = combine_bytes(accel_data[2], accel_data[3]);
            int16_t az = combine_bytes(accel_data[4], accel_data[5]);
            
            pitch_sum += -(atan2f(az, ax) * 180.0f / (float)M_PI);
            roll_sum  +=  (atan2f(ay, ax) * 180.0f / (float)M_PI);
            successful_reads++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    if (successful_reads > 0) {
        g_state.angle_pitch = pitch_sum / successful_reads;
        g_state.angle_roll = roll_sum / successful_reads;
        
        // Use calibrated pitch as setpoint for better initial balance
        g_state.setpoint_angle = g_state.angle_pitch;
        
        ESP_LOGI(TAG, "Initial angles - Pitch: %.2f°, Roll: %.2f°", g_state.angle_pitch, g_state.angle_roll);
        ESP_LOGI(TAG, "Setpoint adjusted to: %.2f°", g_state.setpoint_angle);
    } else {
        ESP_LOGE(TAG, "Angle initialization failed - no successful reads");
    }
    
    ESP_LOGI(TAG, "MPU6050 calibration completed successfully");
}

/* =========================================================================
 * 6. IMPLEMENTASI FUNGSI MOTOR CONTROL
 * ========================================================================= */

static void motor_set_speed(int speed)
{
    // Clamp speed to valid range
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    
    int duty = abs(speed);
    bool reverse = (speed < 0);

    // Apply deadband
    if (duty < MOTOR_DEADBAND) {
        duty = 0;
        // Brake both motors
        gpio_set_level(MOTOR_LEFT_IN1, 1);
        gpio_set_level(MOTOR_LEFT_IN2, 1);
        gpio_set_level(MOTOR_RIGHT_IN1, 1);
        gpio_set_level(MOTOR_RIGHT_IN2, 1);
    } else if (reverse) {
        // Reverse direction
        gpio_set_level(MOTOR_LEFT_IN1, 0);
        gpio_set_level(MOTOR_LEFT_IN2, 1);
        gpio_set_level(MOTOR_RIGHT_IN1, 0);
        gpio_set_level(MOTOR_RIGHT_IN2, 1);
    } else {
        // Forward direction
        gpio_set_level(MOTOR_LEFT_IN1, 1);
        gpio_set_level(MOTOR_LEFT_IN2, 0);
        gpio_set_level(MOTOR_RIGHT_IN1, 1);
        gpio_set_level(MOTOR_RIGHT_IN2, 0);
    }

    // Set PWM duty cycle
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, duty);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT);
}

static void reset_pid(void)
{
    g_state.integral_error = 0.0f;
    g_state.previous_error = 0.0f;
    ESP_LOGI(TAG, "PID controller reset");
}

/* =========================================================================
 * 7. IMPLEMENTASI TASKS
 * ========================================================================= */

static void IRAM_ATTR button_isr_handler(void* arg)
{
    uint32_t button_press = 1;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    xQueueSendFromISR(g_button_queue, &button_press, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void button_task(void* arg)
{
    uint32_t button_press;
    
    while (1) {
        if (xQueueReceive(g_button_queue, &button_press, portMAX_DELAY)) {
            // Simple debounce delay
            vTaskDelay(pdMS_TO_TICKS(50));
            
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                // Toggle motor state
                g_state.motor_active = !g_state.motor_active;
                
                if (g_state.motor_active) {
                    ESP_LOGI(TAG, "MOTORS ACTIVATED");
                    g_state.safety_lock = false;
                    reset_pid();
                } else {
                    ESP_LOGI(TAG, "MOTORS DEACTIVATED");
                    g_state.safety_lock = true;
                    motor_set_speed(0);
                }
                
                // Wait for button release
                while (gpio_get_level(BUTTON_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                
                ESP_LOGI(TAG, "Button released");
            }
        }
    }
}

void balancing_task(void* arg)
{
    uint8_t sensor_data[14];
    int16_t ax, az, gy;
    float accel_pitch, gyro_pitch_rate;
    float dt = (float)LOOP_TIME_MS / 1000.0f;
    
    float pid_output, error, derivative;
    int log_counter = 0;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LOOP_TIME_MS);
    
    ESP_LOGI(TAG, "Balance control loop started (%d ms interval)", LOOP_TIME_MS);

    while (1) {
        // 1. Read and process sensor data
        if (mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, sensor_data, 14) == ESP_OK) {
            ax = combine_bytes(sensor_data[0], sensor_data[1]);
            az = combine_bytes(sensor_data[4], sensor_data[5]);
            gy = combine_bytes(sensor_data[10], sensor_data[11]);
            
            // Calculate angles
            accel_pitch = -(atan2f(az, ax) * 180.0f / (float)M_PI);
            gyro_pitch_rate = -((gy - g_calibration.gyro_y_offset) / GYRO_SCALE_FACTOR);
            
            // Complementary filter
            g_state.angle_pitch = ALPHA * (g_state.angle_pitch + gyro_pitch_rate * dt) + 
                                 (1.0f - ALPHA) * accel_pitch;
        } else {
            ESP_LOGE(TAG, "Failed to read MPU6050 data!");
            g_state.safety_lock = true;
        }

        // 2. Safety logic
        if (!g_state.motor_active) {
            g_state.safety_lock = true;
        } else if (fabsf(g_state.angle_pitch) > MAX_ANGLE_FALL) {
            ESP_LOGW(TAG, "FALL DETECTED! Angle: %.2f° - Locking motors", g_state.angle_pitch);
            g_state.safety_lock = true;
        } else if (g_state.safety_lock && g_state.motor_active && 
                   fabsf(g_state.angle_pitch - g_state.setpoint_angle) < MAX_ANGLE_RECOVER) {
            ESP_LOGI(TAG, "AUTO-RECOVERY: Robot stabilized - Enabling motors");
            g_state.safety_lock = false;
            reset_pid();
        }

        // 3. Motor control
        if (g_state.safety_lock || !g_state.motor_active) {
            motor_set_speed(0);
            reset_pid();
            pid_output = 0;
            error = 0;
            
            // Blink LED when inactive
            gpio_set_level(LED_GPIO, (xTaskGetTickCount() / 250) % 2);
        } else {
            // PID Calculation
            error = g_state.setpoint_angle - g_state.angle_pitch;
            
            g_state.integral_error += error * dt;
            if (g_state.integral_error > INTEGRAL_MAX_CLAMP) g_state.integral_error = INTEGRAL_MAX_CLAMP;
            if (g_state.integral_error < -INTEGRAL_MAX_CLAMP) g_state.integral_error = -INTEGRAL_MAX_CLAMP;
            
            derivative = (error - g_state.previous_error) / dt;
            pid_output = (KP * error) + (KI * g_state.integral_error) + (KD * derivative);
            g_state.previous_error = error;
            
            // Apply motor control (negative for correct direction)
            motor_set_speed((int)-pid_output);
            
            // Solid LED when active
            gpio_set_level(LED_GPIO, 1);
        }

        // 4. Logging
        if (++log_counter >= 2) {
            if (!g_state.safety_lock && g_state.motor_active) {
                // CSV format for analysis
                printf("%.4f,%.4f,%.4f\n", g_state.angle_pitch, error, -pid_output);
            } else {
                ESP_LOGI(TAG, "Pitch: %6.2f° | Setpoint: %6.2f° | Motor: %s | Lock: %d", 
                         g_state.angle_pitch, g_state.setpoint_angle, 
                         g_state.motor_active ? "ON " : "OFF", g_state.safety_lock);
            }
            log_counter = 0;
        }

        // 5. Maintain loop timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* =========================================================================
 * 8. APP_MAIN - APPLICATION ENTRY POINT
 * ========================================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   Self-Balancing Robot - ESP-IDF");
    ESP_LOGI(TAG, "   Optimized and Structured Version");
    ESP_LOGI(TAG, "========================================");

    // 1. Hardware initialization
    if (hardware_init() != ESP_OK) {
        ESP_LOGE(TAG, "Hardware initialization failed! System halted.");
        return;
    }

    // 2. Sensor calibration
    mpu_system_init_and_calibrate();

    // 3. Create tasks
    ESP_LOGI(TAG, "Creating system tasks...");
    
    if (xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task!");
        return;
    }
    
    if (xTaskCreate(balancing_task, "balancing_task", 4096, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create balancing task!");
        return;
    }

    // 4. System ready
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SYSTEM READY");
    ESP_LOGI(TAG, "- Place robot on flat surface");
    ESP_LOGI(TAG, "- Press button (GPIO %d) to start", BUTTON_GPIO);
    ESP_LOGI(TAG, "- Current PID: Kp=%.1f, Ki=%.1f, Kd=%.2f", KP, KI, KD);
    ESP_LOGI(TAG, "========================================");
}