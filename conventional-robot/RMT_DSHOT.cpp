/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "RMT_DSHOT.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "dshot_encoder.h"

#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution
#define DSHOT_ESC_GPIO_NUM_1      GPIO_NUM_1
#define DSHOT_ESC_GPIO_NUM_2      GPIO_NUM_3
#define DSHOT_ESC_GPIO_NUM_3      GPIO_NUM_5

static const char *TAG = "example";

typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47
} dshotCommands_e;

// Motor control structures
// ------------------------

// Shared encoder for all three motors
rmt_encoder_handle_t dshot_encoder = NULL;

// transmit config -infinite loop
rmt_transmit_config_t tx_config = {
    .loop_count = -1, 
};

rmt_channel_handle_t esc_chan[3] = {NULL,NULL, NULL};
dshot_esc_throttle_t throttle[3] = {
    {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    },
    {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    },
    {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    }
};

void initialize_motors()
{
    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config[3] = {
        {
            .gpio_num = DSHOT_ESC_GPIO_NUM_1,
            .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
            .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
            .mem_block_symbols = 48, // NOTE- must be 48 due to hardware design
            .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
        },
        {
            .gpio_num = DSHOT_ESC_GPIO_NUM_2,
            .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
            .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
            .mem_block_symbols = 48,
            .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
        },
        {
            .gpio_num = DSHOT_ESC_GPIO_NUM_3,
            .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
            .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
            .mem_block_symbols = 48,
            .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
        },
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config[0], &esc_chan[0]));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config[1], &esc_chan[1]));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config[2], &esc_chan[2]));

    ESP_LOGI(TAG, "Install Dshot ESC encoder");

    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50, // extra delay between each frame
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(esc_chan[0]));
    ESP_ERROR_CHECK(rmt_enable(esc_chan[1]));
    ESP_ERROR_CHECK(rmt_enable(esc_chan[2]));

    ESP_LOGI(TAG, "Start ESC by sending zero throttle for a while...");
    ESP_ERROR_CHECK(rmt_transmit(esc_chan[0], dshot_encoder, &throttle[0], sizeof(throttle[0]), &tx_config));
    ESP_ERROR_CHECK(rmt_transmit(esc_chan[1], dshot_encoder, &throttle[1], sizeof(throttle[1]), &tx_config));
    ESP_ERROR_CHECK(rmt_transmit(esc_chan[2], dshot_encoder, &throttle[2], sizeof(throttle[2]), &tx_config));
}

void update_throttle(uint16_t motor, uint16_t val){
     throttle[motor].throttle = val;

    ESP_ERROR_CHECK(rmt_transmit(esc_chan[motor], dshot_encoder, &throttle[motor], sizeof(throttle[motor]), &tx_config));
    // the previous loop transfer is till undergoing, we need to stop it and restart,
    // so that the new throttle can be updated on the output
    ESP_ERROR_CHECK(rmt_disable(esc_chan[motor]));
    ESP_ERROR_CHECK(rmt_enable(esc_chan[motor]));
}