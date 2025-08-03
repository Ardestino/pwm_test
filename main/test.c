#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#define MOTOR_COUNT 3
#define MCPWM_RES_HZ 10000000  // 10 MHz
#define STEP_PINS {25, 26, 27}
#define TAG "SYNC_MOTORS"

typedef struct {
    mcpwm_gen_handle_t gen;
    mcpwm_cmpr_handle_t cmp;
    uint32_t target_steps;
    volatile uint32_t current_step;
} motor_control_t;

motor_control_t motors[MOTOR_COUNT];
mcpwm_timer_handle_t shared_timer;
const int step_pins[MOTOR_COUNT] = STEP_PINS;

// ─────────────────────────────────────────────────────────────
// MCPWM setup for timer and all motors (shared timer)
// ─────────────────────────────────────────────────────────────
void setup_mcpwm()
{
    // 1. MCPWM timer (shared)
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .resolution_hz = MCPWM_RES_HZ,
        .period_ticks = 200,  // Temporary, will be set dynamically
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &shared_timer));

    // 2. Create operators and assign to motors
    for (int i = 0; i < MOTOR_COUNT; i++) {
        mcpwm_oper_handle_t oper;
        mcpwm_operator_config_t oper_cfg = {.group_id = 0};
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, shared_timer));

        // 3. Generator (STEP signal)
        mcpwm_generator_config_t gen_cfg = {.gen_gpio_num = step_pins[i]};
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &motors[i].gen));

        // 4. Comparator
        mcpwm_comparator_config_t cmp_cfg = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &motors[i].cmp));
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(shared_timer));
}

// ─────────────────────────────────────────────────────────────
// Configure 50% duty for a generator
// ─────────────────────────────────────────────────────────────
void set_generator_50_percent(int i, uint32_t period_ticks)
{
    uint32_t compare_ticks = period_ticks / 2;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motors[i].cmp, compare_ticks));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        motors[i].gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        motors[i].gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, 
            motors[i].cmp, 
            MCPWM_GEN_ACTION_LOW)
        )
    );
}

// ─────────────────────────────────────────────────────────────
// Movimiento sincronizado: todos los motores terminan juntos
// ─────────────────────────────────────────────────────────────
void synchronized_move(uint32_t step_targets[], uint32_t duration_us)
{
    // 1. Establecer objetivos
    uint32_t max_steps = 0;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i].target_steps = step_targets[i];
        motors[i].current_step = 0;
        if (step_targets[i] > max_steps) max_steps = step_targets[i];
    }

    // 2. Calcular periodo base
    float base_freq = (float)max_steps / (duration_us / 1e6f);
    uint32_t period_ticks = (uint32_t)(MCPWM_RES_HZ / base_freq);
    // ESP_LOGI(TAG, "Sincronizando %d pasos @ %.2f Hz → %u ticks", max_steps, base_freq, period_ticks);

    // 3. Aplicar duty 50% y periodo
    ESP_ERROR_CHECK(mcpwm_timer_set_period(shared_timer, period_ticks));
    for (int i = 0; i < MOTOR_COUNT; i++) {
        set_generator_50_percent(i, period_ticks);
    }

    // 4. Iniciar timer
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(shared_timer, MCPWM_TIMER_START_NO_STOP));

    // 5. Contador de pasos por software
    for (uint32_t tick = 0; tick < max_steps; tick++) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            float ratio = (float)motors[i].target_steps / max_steps;
            if ((uint32_t)(tick * ratio) > motors[i].current_step) {
                motors[i].current_step++;
                // Dejar pulso activo
            } else {
                // Desactivar si ya terminó
                mcpwm_generator_set_force_level(motors[i].gen, 0, true);
            }
        }
        // TODO: ets_delay_us(period_ticks / (MCPWM_RES_HZ / 1000000)); // Delay 1 periodo
    }

    // 6. Detener timer
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(shared_timer, MCPWM_TIMER_STOP_EMPTY));
}

// ─────────────────────────────────────────────────────────────
// Tarea principal: ejecuta una secuencia de movimientos
// ─────────────────────────────────────────────────────────────
void planner_task(void *arg)
{
    setup_mcpwm();
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        uint32_t move1[MOTOR_COUNT] = {1000, 2000, 3000, 4000};
        synchronized_move(move1, 200000); // 200 ms
        vTaskDelay(pdMS_TO_TICKS(300));

        uint32_t move2[MOTOR_COUNT] = {4000, 3000, 2000, 1000};
        synchronized_move(move2, 200000);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// ─────────────────────────────────────────────────────────────
// Entrada principal
// ─────────────────────────────────────────────────────────────
void app_main(void)
{
    xTaskCreate(planner_task, "planner", 8192, NULL, 5, NULL);
}
