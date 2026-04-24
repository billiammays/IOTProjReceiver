/**
 * @file main.c
 * @brief Differential pitch-yaw stepper motor controller with ESP RainMaker cloud control.
 *
 * @details
 * Controls two NEMA17-34 stepper motors (A4988 drivers) arranged in a differential
 * pitch-yaw mechanism. Motor 1 is driven by the ESP32-C6 RMT peripheral; motor 2 is
 * driven by an esp_timer ISR, allowing both axes to step simultaneously.
 *
 * The differential kinematics are:
 * @code
 *   m1 = pitch + yaw
 *   m2 = pitch - yaw   (DIR pin inverted in software for opposing physical mount)
 * @endcode
 *
 * Motion is commanded by writing an integer mode to the ESP RainMaker @c mode
 * parameter from the mobile app or cloud. The motor task polls this value every
 * step cycle (~16 ms), giving sub-step interrupt latency.
 *
 * @par Mode values
 * | Value | Action          |
 * |-------|-----------------|
 * |   0   | Stop            |
 * |   1   | Yaw left        |
 * |   2   | Yaw right       |
 * |   3   | Pitch up        |
 * |   4   | Pitch down      |
 *
 * @par Hardware connections
 * | Signal | GPIO |
 * |--------|------|
 * | STEP 1 |  23  |
 * | DIR 1  |  15  |
 * | STEP 2 |  20  |
 * | DIR 2  |  21  |
 *
 * @author David
 * @date   2025
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_rmaker_core.h"
#include "esp_rmaker_ota.h"
#include "esp_rmaker_mqtt.h"
#include "esp_rmaker_standard_params.h"
#include "esp_rmaker_common_events.h"
#include "app_network.h"
#include "cJSON.h"

/** @brief ESP-IDF log tag. */
static const char *TAG = "STEPPER";

/** @defgroup pins GPIO pin assignments
 *  @{
 */
#define STEP_GPIO_1  23  /**< STEP signal for motor 1 (RMT output). */
#define DIR_GPIO_1   15  /**< DIR signal for motor 1. */
#define STEP_GPIO_2  20  /**< STEP signal for motor 2 (esp_timer output). */
#define DIR_GPIO_2   21  /**< DIR signal for motor 2 (inverted in software). */
/** @} */

/** @defgroup timing Timing constants
 *  @{
 */
#define RMT_RESOLUTION_HZ  1000000  /**< RMT clock resolution: 1 MHz (1 µs per tick). */
#define HALF_PERIOD         8000    /**< RMT/timer half-period in µs → 16 ms/step = 62.5 Hz. */
/** @} */

/** @brief ESP RainMaker node ID of the quaternion transmitter (for reference). */
#define TRANMITTER_NODE_ID "dh4LwEGmot2PhMYSrqPmy5"

/**
 * @brief Active motion mode.
 *
 * Written by @ref write_cb from the RainMaker callback context and read by
 * @ref motor_task every step cycle. Declared @c volatile so the compiler does
 * not cache it in a register across task boundaries.
 *
 * | Value | Meaning     |
 * |-------|-------------|
 * |   0   | Stop        |
 * |   1   | Yaw left    |
 * |   2   | Yaw right   |
 * |   3   | Pitch up    |
 * |   4   | Pitch down  |
 */
static volatile int current_mode = 0;

/** @brief Handle for the RainMaker @c mode parameter. */
static esp_rmaker_param_t *mode_param;

/**
 * @brief Quaternion orientation data received from the transmitter node.
 *
 * Populated by @ref mqtt_callback when a message arrives on the subscribed
 * MQTT topic. Fields follow the Hamilton convention (w, x, y, z).
 */
typedef struct {
    float quat_w;       /**< Scalar (real) component of the quaternion. */
    float quat_x;       /**< X component of the quaternion vector. */
    float quat_y;       /**< Y component of the quaternion vector. */
    float quat_z;       /**< Z component of the quaternion vector. */
    int   active_device;/**< Device index reported by the transmitter. */
} sensor_data_t;

/** @brief Latest sensor data received from the transmitter. */
static sensor_data_t sensor_data = {0};

/** @brief Flag set to @c true by @ref mqtt_callback when new data has arrived. */
static bool new_data_available = false;

/** @name RMT peripheral state
 *  @{
 */
static rmt_channel_handle_t  motor_chan         = NULL; /**< RMT TX channel handle (motor 1). */
static rmt_encoder_handle_t  copy_encoder       = NULL; /**< RMT copy encoder handle. */
static rmt_transmit_config_t transmit_cfg       = { .loop_count = 0 }; /**< Single-shot transmit config. */
static rmt_symbol_word_t     motor1_step_symbol;        /**< Pre-built 50 % duty-cycle step pulse. */
/** @} */

/** @name Motor 2 esp_timer state
 *  @{
 */
static esp_timer_handle_t motor2_timer        = NULL; /**< Periodic timer handle for motor 2 STEP toggling. */
static volatile int       motor2_toggles_left = 0;   /**< Remaining STEP pin toggles; timer stops when zero. */
static volatile bool      motor2_pin          = false;/**< Current logical state of the STEP pin. */
/** @} */

/* ── MQTT Callback ──────────────────────────────────────────────────────────── */

/**
 * @brief MQTT message callback for transmitter sensor data.
 *
 * @details Parses a JSON payload of the form
 * @code{"w":<f>,"x":<f>,"y":<f>,"z":<f>,"d":<i>}@endcode
 * into @ref sensor_data and sets @ref new_data_available.
 *
 * @note Cross-node MQTT subscription is blocked by the AWS IoT ACL on the
 *       RainMaker broker. This callback is retained for future use with an
 *       alternative transport (e.g. ESP-NOW or a local broker).
 *
 * @param[in] topic       MQTT topic string on which the message arrived.
 * @param[in] payload     Raw (non-null-terminated) message payload bytes.
 * @param[in] payload_len Number of bytes in @p payload.
 * @param[in] priv_data   User-supplied context pointer (unused).
 */
static void mqtt_callback(const char *topic, void *payload, size_t payload_len, void *priv_data)
{
    char msg[256] = {0};
    memcpy(msg, payload, payload_len < sizeof(msg) ? payload_len : sizeof(msg) - 1);
    ESP_LOGI(TAG, "Received on %s: %s", topic, msg);
    sscanf(msg, "{\"w\":%f,\"x\":%f,\"y\":%f,\"z\":%f,\"d\":%d}",
       &sensor_data.quat_w,
       &sensor_data.quat_x,
       &sensor_data.quat_y,
       &sensor_data.quat_z,
       &sensor_data.active_device);
    new_data_available = true;
}

/* ── Event Handler ──────────────────────────────────────────────────────────── */

/**
 * @brief Unified ESP event handler for RainMaker, network, and OTA events.
 *
 * @details Logs state transitions for all registered event bases:
 *          - @c RMAKER_EVENT — node lifecycle (init, claim).
 *          - @c RMAKER_COMMON_EVENT — MQTT connectivity and factory-reset commands.
 *          - @c APP_NETWORK_EVENT — BLE provisioning lifecycle.
 *          - @c RMAKER_OTA_EVENT — OTA firmware update progress.
 *
 * @param[in] arg        User-supplied context pointer (unused).
 * @param[in] event_base Event base identifier (e.g. @c RMAKER_EVENT).
 * @param[in] event_id   Event-specific identifier within @p event_base.
 * @param[in] event_data Optional event-specific payload pointer.
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    if (event_base == RMAKER_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_INIT_DONE:
                ESP_LOGI(TAG, "RainMaker Initialised.");
                break;
            case RMAKER_EVENT_CLAIM_STARTED:
                ESP_LOGI(TAG, "RainMaker Claim Started.");
                break;
            case RMAKER_EVENT_CLAIM_SUCCESSFUL:
                ESP_LOGI(TAG, "RainMaker Claim Successful.");
                break;
            case RMAKER_EVENT_CLAIM_FAILED:
                ESP_LOGI(TAG, "RainMaker Claim Failed.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STARTED:
                ESP_LOGI(TAG, "Local Control Started.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STOPPED:
                ESP_LOGI(TAG, "Local Control Stopped.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Event: %"PRIi32, event_id);
        }
    } else if (event_base == RMAKER_COMMON_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_REBOOT:
                ESP_LOGI(TAG, "Rebooting in %d seconds.", *((uint8_t *)event_data));
                break;
            case RMAKER_EVENT_WIFI_RESET:
                ESP_LOGI(TAG, "Wi-Fi credentials reset.");
                break;
            case RMAKER_EVENT_FACTORY_RESET:
                ESP_LOGI(TAG, "Node reset to factory defaults.");
                break;
            case RMAKER_MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT Connected.");
                break;
            case RMAKER_MQTT_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "MQTT Disconnected.");
                break;
            case RMAKER_MQTT_EVENT_PUBLISHED:
                ESP_LOGI(TAG, "MQTT Published. Msg id: %d.", *((int *)event_data));
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Common Event: %"PRIi32, event_id);
        }
    } else if (event_base == APP_NETWORK_EVENT) {
        switch (event_id) {
            case APP_NETWORK_EVENT_QR_DISPLAY:
                ESP_LOGI(TAG, "Provisioning QR : %s", (char *)event_data);
                break;
            case APP_NETWORK_EVENT_PROV_TIMEOUT:
                ESP_LOGI(TAG, "Provisioning Timed Out. Please reboot.");
                break;
            case APP_NETWORK_EVENT_PROV_RESTART:
                ESP_LOGI(TAG, "Provisioning has restarted due to failures.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled App Wi-Fi Event: %"PRIi32, event_id);
                break;
        }
    } else if (event_base == RMAKER_OTA_EVENT) {
        switch (event_id) {
            case RMAKER_OTA_EVENT_STARTING:
                ESP_LOGI(TAG, "Starting OTA.");
                break;
            case RMAKER_OTA_EVENT_IN_PROGRESS:
                ESP_LOGI(TAG, "OTA is in progress.");
                break;
            case RMAKER_OTA_EVENT_SUCCESSFUL:
                ESP_LOGI(TAG, "OTA successful.");
                break;
            case RMAKER_OTA_EVENT_FAILED:
                ESP_LOGI(TAG, "OTA Failed.");
                break;
            case RMAKER_OTA_EVENT_REJECTED:
                ESP_LOGI(TAG, "OTA Rejected.");
                break;
            case RMAKER_OTA_EVENT_DELAYED:
                ESP_LOGI(TAG, "OTA Delayed.");
                break;
            case RMAKER_OTA_EVENT_REQ_FOR_REBOOT:
                ESP_LOGI(TAG, "Firmware image downloaded. Please reboot to apply.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled OTA Event: %"PRIi32, event_id);
                break;
        }
    } else {
        ESP_LOGW(TAG, "Invalid event received!");
    }
}

/* ── Motor 2 (esp_timer) ────────────────────────────────────────────────────── */

/**
 * @brief esp_timer ISR callback: toggles the motor 2 STEP pin.
 *
 * @details Called at @c HALF_PERIOD intervals. Each pair of toggles (high then
 *          low) constitutes one complete step pulse. The timer is stopped
 *          automatically after the requested number of steps completes.
 *
 * @note Executes in ISR context — must not call blocking FreeRTOS APIs.
 *
 * @param[in] arg User-supplied context pointer (unused).
 */
static void motor2_step_cb(void *arg)
{
    motor2_pin = !motor2_pin;
    gpio_set_level(STEP_GPIO_2, motor2_pin);
    if (--motor2_toggles_left == 0) {
        gpio_set_level(STEP_GPIO_2, 0);
        esp_timer_stop(motor2_timer);
    }
}

/**
 * @brief Start motor 2 for a given number of steps.
 *
 * @details Configures @ref motor2_toggles_left and starts the periodic timer.
 *          Returns immediately; use @ref motor2_wait to synchronise.
 *
 * @param[in] steps Number of full steps to execute (each step = 2 timer periods).
 */
static void motor2_start(int steps)
{
    motor2_pin          = false;
    motor2_toggles_left = steps * 2;
    esp_timer_start_periodic(motor2_timer, HALF_PERIOD);
}

/**
 * @brief Block until motor 2 finishes its current step sequence.
 *
 * @details Polls @ref motor2_toggles_left in 1 ms increments. Safe to call
 *          even if motor 2 is not running (@ref motor2_toggles_left will
 *          already be zero).
 */
static void motor2_wait(void)
{
    while (motor2_toggles_left > 0) vTaskDelay(pdMS_TO_TICKS(1));
}

/* ── Motor task ─────────────────────────────────────────────────────────────── */

/**
 * @brief FreeRTOS task: drives both stepper motors continuously.
 *
 * @details Reads @ref current_mode on every step cycle (~16 ms). Direction pins
 *          are updated only on mode transitions, followed by a 5 ms settle delay
 *          required by the A4988 driver before the first step pulse.
 *
 *          Both motors step simultaneously: motor 1 via the RMT peripheral,
 *          motor 2 via the esp_timer ISR. The task blocks on
 *          @c rmt_tx_wait_all_done until motor 1 finishes, then calls
 *          @ref motor2_wait for motor 2.
 *
 * @par DIR pin truth table (DIR_GPIO_2 is inverted for opposing mount)
 * | Mode | m1 | m2 | DIR1 | DIR2 |
 * |------|----|----|------|------|
 * |  1   | +1 | -1 |  1   |  1   |
 * |  2   | -1 | +1 |  0   |  0   |
 * |  3   | +1 | +1 |  1   |  0   |
 * |  4   | -1 | -1 |  0   |  1   |
 *
 * @param[in] arg FreeRTOS task parameter (unused).
 */
static void motor_task(void *arg)
{
    int last_mode = -1;

    while (1) {
        int mode = current_mode;

        if (mode == 0) {
            last_mode = 0;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (mode != last_mode) {
            switch (mode) {
                case 1: gpio_set_level(DIR_GPIO_1, 1); gpio_set_level(DIR_GPIO_2, 1); break;
                case 2: gpio_set_level(DIR_GPIO_1, 0); gpio_set_level(DIR_GPIO_2, 0); break;
                case 3: gpio_set_level(DIR_GPIO_1, 1); gpio_set_level(DIR_GPIO_2, 0); break;
                case 4: gpio_set_level(DIR_GPIO_1, 0); gpio_set_level(DIR_GPIO_2, 1); break;
            }
            vTaskDelay(pdMS_TO_TICKS(5)); // DIR settle
            last_mode = mode;
        }

        motor2_start(1);
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, copy_encoder,
                                     &motor1_step_symbol, sizeof(rmt_symbol_word_t),
                                     &transmit_cfg));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
        motor2_wait();
    }
}

/* ── RainMaker write callback ───────────────────────────────────────────────── */

/**
 * @brief RainMaker device write callback: handles incoming @c mode parameter updates.
 *
 * @details Validates the received integer, updates @ref current_mode, and
 *          acknowledges the new value back to the cloud via
 *          @c esp_rmaker_param_update_and_report. Updates to any parameter
 *          other than @c "mode" are silently ignored.
 *
 * @param[in]  device    RainMaker device handle.
 * @param[in]  param     Parameter handle that was written.
 * @param[in]  val       New parameter value (integer, 0–4).
 * @param[in]  priv_data User-supplied context pointer (unused).
 * @param[in]  ctx       Write context (source: local / remote / init).
 *
 * @retval ESP_OK              Mode accepted and applied.
 * @retval ESP_ERR_INVALID_ARG Mode value outside the valid range [0, 4].
 */
static esp_err_t write_cb(const esp_rmaker_device_t *device,
                           const esp_rmaker_param_t  *param,
                           const esp_rmaker_param_val_t val,
                           void *priv_data,
                           esp_rmaker_write_ctx_t *ctx)
{
    if (strcmp(esp_rmaker_param_get_name(param), "mode") != 0)
        return ESP_OK;

    int mode = val.val.i;
    if (mode < 0 || mode > 4) {
        ESP_LOGW(TAG, "Invalid mode: %d", mode);
        return ESP_ERR_INVALID_ARG;
    }

    current_mode = mode;
    ESP_LOGI(TAG, "Mode set to %d", mode);
    esp_rmaker_param_update_and_report(param, val);
    return ESP_OK;
}

/* ── RainMaker / network QR event handler ───────────────────────────────────── */

/**
 * @brief Secondary event handler: logs the BLE provisioning QR code payload.
 *
 * @details Fires on @c APP_NETWORK_EVENT_QR_DISPLAY and prints the raw JSON
 *          payload string. The Espressif RainMaker app scans this QR code to
 *          complete BLE provisioning on first boot.
 *
 * @param[in] arg  User-supplied context pointer (unused).
 * @param[in] base Event base (expected: @c APP_NETWORK_EVENT).
 * @param[in] id   Event ID (expected: @c APP_NETWORK_EVENT_QR_DISPLAY).
 * @param[in] data QR payload string (null-terminated JSON).
 */
static void rmaker_event_handler(void *arg, esp_event_base_t base,
                                  int32_t id, void *data)
{
    if (base == APP_NETWORK_EVENT && id == APP_NETWORK_EVENT_QR_DISPLAY) {
        ESP_LOGI(TAG, "Scan this QR code from the ESP RainMaker phone app for Provisioning.");
        ESP_LOGI(TAG, "If QR code is not visible, copy paste the below URL in a browser.\n"
                      "https://rainmaker.espressif.com/qrcode.html?data=%s", (char *)data);
    }
}

/* ── Hardware initialisation ────────────────────────────────────────────────── */

/**
 * @brief Initialise GPIO, RMT, and esp_timer peripherals.
 *
 * @details Performs the following in order:
 *          1. Configures DIR GPIO pins as outputs.
 *          2. Configures the motor 2 STEP GPIO as an output.
 *          3. Creates the motor 2 esp_timer (not started here).
 *          4. Creates and enables the RMT TX channel for motor 1 STEP.
 *          5. Creates the RMT copy encoder.
 *          6. Pre-builds @ref motor1_step_symbol (50 % duty cycle at @c HALF_PERIOD).
 */
static void hw_init(void)
{
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << DIR_GPIO_1) | (1ULL << DIR_GPIO_2),
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    gpio_config(&io_conf);

    gpio_reset_pin(STEP_GPIO_2);
    gpio_set_direction(STEP_GPIO_2, GPIO_MODE_OUTPUT);

    esp_timer_create_args_t timer_args = { .callback = motor2_step_cb, .name = "motor2" };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &motor2_timer));

    rmt_tx_channel_config_t tx_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = STEP_GPIO_1,
        .mem_block_symbols = 128,
        .resolution_hz     = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &motor_chan));
    ESP_ERROR_CHECK(rmt_enable(motor_chan));

    rmt_copy_encoder_config_t enc_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &copy_encoder));

    motor1_step_symbol.duration0 = HALF_PERIOD;
    motor1_step_symbol.level0    = 1;
    motor1_step_symbol.duration1 = HALF_PERIOD;
    motor1_step_symbol.level1    = 0;
}

/* ── app_main ───────────────────────────────────────────────────────────────── */

/**
 * @brief Application entry point.
 *
 * @details Initialisation sequence:
 *          1. NVS flash (erases and reinitialises on version mismatch).
 *          2. Default FreeRTOS event loop.
 *          3. Hardware peripherals via @ref hw_init.
 *          4. @ref motor_task FreeRTOS task (priority 5, 4 kB stack).
 *          5. Event handler registration for RainMaker, network, and OTA events.
 *          6. @c app_network_init — must precede @c esp_rmaker_node_init so the
 *             Wi-Fi MAC address is available for node ID derivation.
 *          7. RainMaker node, device, and @c mode parameter creation.
 *          8. @c esp_rmaker_start — starts the RainMaker agent task.
 *          9. @c app_network_start — blocks until Wi-Fi is connected (BLE
 *             provisioning on first boot, stored credentials thereafter).
 *
 * After initialisation, the main task polls @ref new_data_available in a
 * 10 ms loop, logging any data received via @ref mqtt_callback.
 */
void app_main(void)
{
    // 1. NVS + event loop
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 2. Hardware + motor task
    hw_init();
    xTaskCreate(motor_task, "motor", 4096, NULL, 5, NULL);

    // 3. Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_EVENT,        ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_NETWORK_EVENT,   ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_OTA_EVENT,    ESP_EVENT_ANY_ID, &event_handler, NULL));

    // 4. WiFi init — must come before esp_rmaker_node_init (needs MAC address)
    app_network_init();

    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_EVENT,        ESP_EVENT_ANY_ID, rmaker_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, rmaker_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_NETWORK_EVENT,   ESP_EVENT_ANY_ID, rmaker_event_handler, NULL));

    // 5. Rainmaker node + device
    esp_rmaker_config_t rmaker_cfg = { .enable_time_sync = false };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rmaker_cfg, "PitchYaw Node", "Controller");
    ESP_ERROR_CHECK(!node ? ESP_FAIL : ESP_OK);

    esp_rmaker_device_t *device = esp_rmaker_device_create("PitchYaw", NULL, NULL);
    ESP_ERROR_CHECK(!device ? ESP_FAIL : ESP_OK);

    mode_param = esp_rmaker_param_create(
        "mode", NULL,
        esp_rmaker_int(0),
        PROP_FLAG_READ | PROP_FLAG_WRITE
    );
    ESP_ERROR_CHECK(esp_rmaker_device_add_param(device, mode_param));
    ESP_ERROR_CHECK(esp_rmaker_device_add_cb(device, write_cb, NULL));
    ESP_ERROR_CHECK(esp_rmaker_node_add_device(node, device));

    // 6. Start Rainmaker agent
    ESP_ERROR_CHECK(esp_rmaker_start());

    // 7. Start WiFi — provisions on first boot, connects on subsequent boots
    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start WiFi. Aborting.");
        abort();
    }

    while (1) {
        if (new_data_available == true) {
            new_data_available = false;
            ESP_LOGI(TAG, "New data: w=%.3f x=%.3f y=%.3f z=%.3f device=%d",
                sensor_data.quat_w,
                sensor_data.quat_x,
                sensor_data.quat_y,
                sensor_data.quat_z,
                sensor_data.active_device);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
