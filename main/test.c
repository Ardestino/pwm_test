#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#define MOTOR_COUNT 3
#define MCPWM_RES_HZ 1000000 // 1 MHz resolución
#define STEP_PINS {27, 33, 19}
#define TAG "SYNC_TIMERS"

typedef struct
{
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_gen_handle_t gen;
    mcpwm_cmpr_handle_t cmp;
    uint32_t target_steps;
    uint32_t step_count;
    bool active;
} motor_control_t;

motor_control_t motors[MOTOR_COUNT];
const int step_pins[MOTOR_COUNT] = STEP_PINS;

static bool IRAM_ATTR motor_timer_callback(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{
    motor_control_t *motor = (motor_control_t *)user_ctx;

    motor->step_count++;
    if (motor->step_count >= motor->target_steps)
    {
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(motor->timer, MCPWM_TIMER_STOP_EMPTY));

        // ESP_LOGI(TAG, "Movimiento completado");

        mcpwm_generator_set_force_level(motor->gen, 0, true);
        motor->active = false;
        return true; // Stop timer
    }
    return false;
}

void setup_motor(int index, float frequency_hz)
{
    motor_control_t *motor = &motors[index];
    uint32_t period_ticks = MCPWM_RES_HZ / frequency_hz;

    // Timer individual para cada motor
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_RES_HZ,
        .period_ticks = period_ticks,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &motor->timer));

    // Operador
    mcpwm_operator_config_t oper_cfg = {.group_id = 0};
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &motor->oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motor->oper, motor->timer));

    // Generador
    mcpwm_generator_config_t gen_cfg = {.gen_gpio_num = step_pins[index]};
    ESP_ERROR_CHECK(mcpwm_new_generator(motor->oper, &gen_cfg, &motor->gen));

    // Comparador
    mcpwm_comparator_config_t cmp_cfg = {.flags.update_cmp_on_tez = true};
    ESP_ERROR_CHECK(mcpwm_new_comparator(motor->oper, &cmp_cfg, &motor->cmp));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor->cmp, period_ticks / 2));

    // Generar pulso 50%
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        motor->gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        motor->gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->cmp, MCPWM_GEN_ACTION_LOW)));

    // Registrar callback del timer
    mcpwm_timer_event_callbacks_t cbs = {
        .on_full = motor_timer_callback,
    };
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(motor->timer, &cbs, motor));
}

void cleanup_motor(int index)
{
    motor_control_t *motor = &motors[index];
    
    if (motor->timer) {
        mcpwm_timer_disable(motor->timer);
        mcpwm_del_timer(motor->timer);
        motor->timer = NULL;
    }
    
    if (motor->gen) {
        mcpwm_del_generator(motor->gen);
        motor->gen = NULL;
    }
    
    if (motor->cmp) {
        mcpwm_del_comparator(motor->cmp);
        motor->cmp = NULL;
    }
    
    if (motor->oper) {
        mcpwm_del_operator(motor->oper);
        motor->oper = NULL;
    }
}

void start_synchronized_move(uint32_t steps[], float duration_sec)
{
    ESP_LOGI(TAG, "== Iniciando movimiento sincronizado ==");
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        motors[i].target_steps = steps[i];
        motors[i].step_count = 0;
        motors[i].active = steps[i] > 0;

        if (motors[i].active)
        {
            float freq = steps[i] / duration_sec;
            ESP_LOGI(TAG, "Motor %d: %lu pasos, %.2f Hz", i, steps[i], freq);
            setup_motor(i, freq);
        }
    }

    // Habilitar timers pero sin iniciar aún
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (motors[i].active)
        {
            ESP_ERROR_CHECK(mcpwm_timer_enable(motors[i].timer));
        }
    }

    // Iniciar todos al mismo tiempo
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (motors[i].active)
        {
            ESP_ERROR_CHECK(mcpwm_timer_start_stop(motors[i].timer, MCPWM_TIMER_START_NO_STOP));
        }
    }

    // Esperar a que todos terminen
    bool all_done = false;
    while (!all_done)
    {
        all_done = true;
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            if (motors[i].active)
            {
                all_done = false;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Limpiar recursos después del movimiento
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        cleanup_motor(i);
    }

    ESP_LOGI(TAG, "== Movimiento finalizado ==");
}

void planner_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1)
    {
        uint32_t move1[MOTOR_COUNT] = {1000, 2000, 3000};
        start_synchronized_move(move1, 2.0f); // 2 segundos

        vTaskDelay(pdMS_TO_TICKS(3000));

        uint32_t move2[MOTOR_COUNT] = {3000, 1500, 500};
        start_synchronized_move(move2, 2.0f);

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void app_main(void)
{
    xTaskCreate(planner_task, "planner", 8192, NULL, 5, NULL);
}
