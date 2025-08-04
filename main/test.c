#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define MOTOR_COUNT 3
#define MCPWM_RES_HZ 1000000 // 1 MHz resolución
#define STEP_PINS {27, 33, 19}
#define DIR_PINS {26, 32, 21} // Pines de dirección para cada motor
#define TAG "SYNC_TIMERS"

// Estructura para definir un movimiento
typedef struct
{
    int32_t steps[MOTOR_COUNT]; // Pasos con signo (+ o -)
    float duration_sec;
    uint32_t delay_after_ms; // Pausa después del movimiento
} movement_t;

typedef struct
{
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_gen_handle_t gen;
    mcpwm_cmpr_handle_t cmp;
    uint32_t target_steps;
    uint32_t step_count;
    bool active;
    bool direction; // true = positivo, false = negativo
} motor_control_t;

motor_control_t motors[MOTOR_COUNT];
const int step_pins[MOTOR_COUNT] = STEP_PINS;
const int dir_pins[MOTOR_COUNT] = DIR_PINS;

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

void init_direction_pins(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        gpio_config_t dir_conf = {
            .pin_bit_mask = (1ULL << dir_pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&dir_conf));
        gpio_set_level(dir_pins[i], 0); // Inicializar en LOW
    }
}

void set_motor_direction(int motor_index, bool direction)
{
    // true = dirección positiva (HIGH), false = dirección negativa (LOW)
    gpio_set_level(dir_pins[motor_index], direction ? 1 : 0);
    motors[motor_index].direction = direction;
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

    if (motor->timer)
    {
        mcpwm_timer_disable(motor->timer);
        mcpwm_del_timer(motor->timer);
        motor->timer = NULL;
    }

    if (motor->gen)
    {
        mcpwm_del_generator(motor->gen);
        motor->gen = NULL;
    }

    if (motor->cmp)
    {
        mcpwm_del_comparator(motor->cmp);
        motor->cmp = NULL;
    }

    if (motor->oper)
    {
        mcpwm_del_operator(motor->oper);
        motor->oper = NULL;
    }
}

void start_synchronized_move(int32_t steps[], float duration_sec)
{
    ESP_LOGI(TAG, "== Iniciando movimiento sincronizado ==");
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        // Determinar dirección y pasos absolutos
        bool direction = steps[i] >= 0;
        uint32_t abs_steps = abs(steps[i]);

        motors[i].target_steps = abs_steps;
        motors[i].step_count = 0;
        motors[i].active = abs_steps > 0;
        motors[i].direction = direction;

        if (motors[i].active)
        {
            // Configurar dirección
            set_motor_direction(i, direction);

            float freq = abs_steps / duration_sec;
            ESP_LOGI(TAG, "Motor %d: %ld pasos (%s), %.2f Hz",
                     i, steps[i], direction ? "+" : "-", freq);
            setup_motor(i, freq);
        }
        else
        {
            ESP_LOGI(TAG, "Motor %d: Sin movimiento", i);
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

void execute_movement_sequence(movement_t movements[], int num_movements)
{
    ESP_LOGI(TAG, "=== Iniciando secuencia de %d movimientos ===", num_movements);

    for (int i = 0; i < num_movements; i++)
    {
        ESP_LOGI(TAG, "Ejecutando movimiento %d/%d", i + 1, num_movements);
        ESP_LOGI(TAG, "Pasos: [%ld, %ld, %ld], Duración: %.2fs",
                 movements[i].steps[0], movements[i].steps[1], movements[i].steps[2],
                 movements[i].duration_sec);

        start_synchronized_move(movements[i].steps, movements[i].duration_sec);

        if (movements[i].delay_after_ms > 0)
        {
            ESP_LOGI(TAG, "Pausa de %lu ms", movements[i].delay_after_ms);
            vTaskDelay(pdMS_TO_TICKS(movements[i].delay_after_ms));
        }
    }

    ESP_LOGI(TAG, "=== Secuencia completada ===");
}

void planner_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));

    // Inicializar pines de dirección
    init_direction_pins();
    ESP_LOGI(TAG, "Pines de dirección inicializados");

    // Secuencia de subida completa del robot
    movement_t sequence1[] = {
        {{0, 2100, 0}, 0.5f, 1000},     // Movimiento 1: Q2 Full positivo
        {{0, 0, 700}, 0.5f, 1000},      // Movimiento 2: Q3 Full positivo
        {{3200, 0, 0}, 0.5f, 1000},     // Movimiento 3: Q1 Full positivo
        {{-1600, 0, -350}, 1.0f, 1000}, // Movimiento 4: Q1 y Q3 hacia atrás
        {{0, -1050, 0}, 0.5f, 1000},    // Movimiento 5: Q2 hacia atrás
    };
    int num_movements = sizeof(sequence1) / sizeof(movement_t);

    // Subir el robot
    execute_movement_sequence(sequence1, num_movements);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100)); // Mantener la tarea activa
    }
}

void app_main(void)
{
    xTaskCreate(planner_task, "planner", 8192, NULL, 5, NULL);
}
