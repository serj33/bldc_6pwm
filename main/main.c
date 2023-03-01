#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>
#include <driver/gpio.h>

#define Y_DRIVE_IN_PIN 33
#define Y_DRIVE_SD_PIN 25

#define B_DRIVE_IN_PIN 27
#define B_DRIVE_SD_PIN 26

#define G_DRIVE_IN_PIN 12
#define G_DRIVE_SD_PIN 14

#define Y_HALL_S_PIN 21
#define B_HALL_S_PIN 22
#define G_HALL_S_PIN 23

typedef enum {
    PHASE_Z,
    PHASE_0,
    PHASE_1
} phase_state;

const phase_state BRIDGE_STATE[8][3] = {
    [0] = {PHASE_Z, PHASE_Z, PHASE_Z},
    [1] = {PHASE_1, PHASE_Z, PHASE_0},
    [2] = {PHASE_0, PHASE_1, PHASE_Z},
    [3] = {PHASE_Z, PHASE_1, PHASE_0},
    [4] = {PHASE_Z, PHASE_0, PHASE_1},
    [5] = {PHASE_1, PHASE_0, PHASE_Z},
    [6] = {PHASE_0, PHASE_Z, PHASE_1},
    [7] = {PHASE_Z, PHASE_Z, PHASE_Z}
};

void update_bridge(phase_state y_phase, phase_state b_phase, phase_state g_phase) {
    switch(y_phase){
    case PHASE_Z:
        gpio_set_level(Y_DRIVE_SD_PIN, 0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_HAL_GENERATOR_MODE_FORCE_LOW);
        break;
    case PHASE_0:
        gpio_set_level(Y_DRIVE_SD_PIN, 1);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_HAL_GENERATOR_MODE_FORCE_LOW);
        break;
    case PHASE_1:
        gpio_set_level(Y_DRIVE_SD_PIN, 1);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
        break;
    default:
        break;
    }

    switch(b_phase){
    case PHASE_Z:
        gpio_set_level(B_DRIVE_SD_PIN, 0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_HAL_GENERATOR_MODE_FORCE_LOW);
        break;
    case PHASE_0:
        gpio_set_level(B_DRIVE_SD_PIN, 1);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_HAL_GENERATOR_MODE_FORCE_LOW);
        break;
    case PHASE_1:
        gpio_set_level(B_DRIVE_SD_PIN, 1);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
        break;
    default:
        break;
    }

    switch(g_phase){
    case PHASE_Z:
        gpio_set_level(G_DRIVE_SD_PIN, 0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, MCPWM_HAL_GENERATOR_MODE_FORCE_LOW);
        break;
    case PHASE_0:
        gpio_set_level(G_DRIVE_SD_PIN, 1);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, MCPWM_HAL_GENERATOR_MODE_FORCE_LOW);
        break;
    case PHASE_1:
        gpio_set_level(G_DRIVE_SD_PIN, 1);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
        break;
    default:
        break;
    }

}

void IRAM_ATTR mcpwm_cap_it(void* arg) {
    if(MCPWM0.int_raw.cap0_int_raw == 1) { 
        mcpwm_capture_signal_get_edge(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
        static uint32_t hall_period_cnt = 0;
        static uint32_t full_turn_us = 0;
        static int64_t pos_edge_ts = 0;
        if(gpio_get_level(Y_HALL_S_PIN) != 1) {
            uint32_t single_period_us = (uint32_t)(esp_timer_get_time() - pos_edge_ts);
            full_turn_us += single_period_us;
            if(hall_period_cnt++ >= 15 - 1) {
                ets_printf("rpm: %d\n", (1000000 * 60) / full_turn_us);
                hall_period_cnt = 0;
                full_turn_us = 0;
            }
        } else {
            pos_edge_ts = esp_timer_get_time();
        }
    }
    mcpwm_capture_signal_get_edge(MCPWM_UNIT_0, MCPWM_SELECT_CAP1);
    mcpwm_capture_signal_get_edge(MCPWM_UNIT_0, MCPWM_SELECT_CAP2);


    int Y = gpio_get_level(Y_HALL_S_PIN);
    int B = gpio_get_level(B_HALL_S_PIN);
    int G = gpio_get_level(G_HALL_S_PIN);

    // char s[20];
    // sprintf(s, "Y:%d, B:%d, G:%d\n", Y , B, G);
    // ets_printf(s);

    int hall_code = ((Y & 1) << 0) |  ((B & 1) << 1) |  ((G & 1) << 2);
    update_bridge(BRIDGE_STATE[hall_code][0], BRIDGE_STATE[hall_code][1], BRIDGE_STATE[hall_code][2]);
}

void app_main(void) {
    //config drivers sd pins
    gpio_config_t pins_cfg = {
        .pin_bit_mask = BIT(Y_DRIVE_SD_PIN) | BIT(B_DRIVE_SD_PIN) | BIT(G_DRIVE_SD_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&pins_cfg);

    //setup phases pwm
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, Y_DRIVE_IN_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, B_DRIVE_IN_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, G_DRIVE_IN_PIN);

    mcpwm_config_t mcpwm_cfg = {
        .frequency = 8000,
        .cmpr_a = 10,
        .cmpr_b = 10,
        .duty_mode = MCPWM_HAL_GENERATOR_MODE_FORCE_LOW,// MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &mcpwm_cfg);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &mcpwm_cfg);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &mcpwm_cfg);

    //setup capture
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, Y_HALL_S_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, B_HALL_S_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, G_HALL_S_PIN);

    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_BOTH_EDGE, 0);
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_BOTH_EDGE, 0);
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_BOTH_EDGE, 0);

    mcpwm_isr_register(MCPWM_UNIT_0, mcpwm_cap_it, NULL, ESP_INTR_FLAG_IRAM, NULL);
    MCPWM0.int_ena.cap0_int_ena = 1;
    MCPWM0.int_ena.cap1_int_ena = 1;
    MCPWM0.int_ena.cap2_int_ena = 1;
}