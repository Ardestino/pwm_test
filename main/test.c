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
    float step_interval;  // Intervalo entre pasos para este motor
    volatile float next_step_time;  // Tiempo para el próximo paso
    bool active;
} motor_control_t;

motor_control_t motors[MOTOR_COUNT];
mcpwm_timer_handle_t shared_timer;
const int step_pins[MOTOR_COUNT] = STEP_PINS;
volatile uint32_t timer_counter = 0;

// Callback del timer - ahora maneja diferentes frecuencias por motor
static bool IRAM_ATTR timer_callback(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{
    timer_counter++;
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i].active) {
            // Verificar si es tiempo de dar el siguiente paso
            if (timer_counter >= motors[i].next_step_time) {
                motors[i].current_step++;
                motors[i].next_step_time += motors[i].step_interval;
                
                // Verificar si el motor completó sus pasos
                if (motors[i].current_step >= motors[i].target_steps) {
                    motors[i].active = false;
                    mcpwm_generator_set_force_level(motors[i].gen, 0, true);
                }
            }
        }
    }
    
    // Verificar si todos los motores terminaron
    bool any_active = false;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i].active) {
            any_active = true;
            break;
        }
    }
    
    if (!any_active) {
        timer_counter = 0;
        return true; // Stop timer
    }
    
    return false;
}

void setup_mcpwm()
{
    // 1. MCPWM timer (shared) - frecuencia base alta para precisión
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .resolution_hz = MCPWM_RES_HZ,
        .period_ticks = 100,  // 100kHz de frecuencia base para precisión
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &shared_timer));

    // Registrar callback del timer
    mcpwm_timer_event_callbacks_t timer_cbs = {
        .on_empty = timer_callback,
    };
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(shared_timer, &timer_cbs, NULL));

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
        
        motors[i].active = false;
        motors[i].current_step = 0;
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(shared_timer));
}

void set_generator_50_percent(int i, uint32_t period_ticks)
{
    uint32_t compare_ticks = period_ticks / 2;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motors[i].cmp, compare_ticks));

    // Reactivar el generador (en caso de que esté forzado)
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(motors[i].gen, -1, true));

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

void synchronized_move(uint32_t step_targets[], uint32_t duration_us)
{
    // 1. Configurar cada motor con su propia frecuencia
    uint32_t max_steps = 0;
    const float base_timer_freq = 100000.0f; // 100kHz timer base
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i].target_steps = step_targets[i];
        motors[i].current_step = 0;
        motors[i].active = (step_targets[i] > 0);
        
        if (step_targets[i] > max_steps) max_steps = step_targets[i];
        
        if (motors[i].active) {
            // Calcular frecuencia específica para este motor
            float motor_freq = (float)step_targets[i] * 1000000.0f / duration_us;
            // Intervalo entre pasos en términos de ticks del timer base
            motors[i].step_interval = base_timer_freq / motor_freq;
            motors[i].next_step_time = motors[i].step_interval;
            
            ESP_LOGI(TAG, "Motor %d: %lu pasos, %.2f Hz, intervalo %.2f ticks", 
                     i, step_targets[i], motor_freq, motors[i].step_interval);
        }
    }

    if (max_steps == 0) return;
    
    ESP_LOGI(TAG, "Movimiento sincronizado: %lu pasos máx, duración %lu us", max_steps, duration_us);

    // 2. Configurar generadores con periodo fijo pequeño
    uint32_t period_ticks = 50; // Periodo corto para pulsos rápidos
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i].active) {
            set_generator_50_percent(i, period_ticks);
        } else {
            mcpwm_generator_set_force_level(motors[i].gen, 0, true);
        }
    }

    // 3. Reset contador y iniciar movimiento
    timer_counter = 0;
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(shared_timer, MCPWM_TIMER_START_NO_STOP));
    
    // 4. Esperar a que termine
    while (true) {
        bool any_active = false;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (motors[i].active) {
                any_active = true;
                break;
            }
        }
        if (!any_active) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // 5. Detener timer
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(shared_timer, MCPWM_TIMER_STOP_EMPTY));
    
    ESP_LOGI(TAG, "Movimiento completado");
}

void planner_task(void *arg)
{
    setup_mcpwm();
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        ESP_LOGI(TAG, "=== Movimiento 1 ===");
        uint32_t move1[MOTOR_COUNT] = {1000, 2000, 3000};
        synchronized_move(move1, 2000000); // 2 segundos para ver mejor
        
        // Asegurar que todos los motores estén en bajo
        ESP_LOGI(TAG, "Forzando todos los motores a LOW...");
        for (int i = 0; i < MOTOR_COUNT; i++) {
            mcpwm_generator_set_force_level(motors[i].gen, 0, true);
        }
        
        // Delay largo para comparar visualmente
        ESP_LOGI(TAG, "Esperando 3 segundos con pulsos en BAJO...");
        vTaskDelay(pdMS_TO_TICKS(3000));

        ESP_LOGI(TAG, "=== Movimiento 2 ===");
        uint32_t move2[MOTOR_COUNT] = {3000, 1500, 500};
        synchronized_move(move2, 2000000); // Diferentes pasos, mismo tiempo
        
        // Asegurar que todos los motores estén en bajo
        ESP_LOGI(TAG, "Forzando todos los motores a LOW...");
        for (int i = 0; i < MOTOR_COUNT; i++) {
            mcpwm_generator_set_force_level(motors[i].gen, 0, true);
        }
        
        // Delay largo para comparar visualmente
        ESP_LOGI(TAG, "Esperando 3 segundos con pulsos en BAJO...");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void app_main(void)
{
    xTaskCreate(planner_task, "planner", 8192, NULL, 5, NULL);
}