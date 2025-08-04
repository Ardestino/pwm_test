#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_log.h"

constexpr int MOTOR_COUNT = 3;
constexpr uint32_t MCPWM_RES_HZ = 1000000; // 1 MHz resolución
constexpr std::array<int, MOTOR_COUNT> STEP_PINS = {27, 33, 19};
constexpr std::array<int, MOTOR_COUNT> DIR_PINS = {26, 32, 21};

// Límites de movimiento para cada motor (en pasos)
constexpr std::array<int32_t, MOTOR_COUNT> MIN_LIMITS = {0, 0, 0}; // Límites mínimos
constexpr std::array<int32_t, MOTOR_COUNT> MAX_LIMITS = {3200, 2100, 700}; // Límites máximos

constexpr const char *TAG = "SYNC_TIMERS_CPP";

// Estructura para definir un movimiento
struct Movement
{
    std::array<int32_t, MOTOR_COUNT> steps; // Pasos con signo (+ o -)
    float duration_sec;
    uint32_t delay_after_ms; // Pausa después del movimiento

    Movement(std::array<int32_t, MOTOR_COUNT> s, float d, uint32_t delay = 0)
        : steps(s), duration_sec(d), delay_after_ms(delay) {}
};

// Estructura para rastrear la posición del robot
struct RobotPosition
{
    std::array<int32_t, MOTOR_COUNT> current_position = {0, 0, 0}; // Posición actual de cada motor

    void update_position(int motor_index, int32_t steps_moved)
    {
        current_position[motor_index] += steps_moved;
    }

    bool is_move_valid(const std::array<int32_t, MOTOR_COUNT> &target_steps) const
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            int32_t new_position = current_position[i] + target_steps[i];
            if (new_position < MIN_LIMITS[i] || new_position > MAX_LIMITS[i])
            {
                return false;
            }
        }
        return true;
    }

    std::array<int32_t, MOTOR_COUNT> get_limited_steps(const std::array<int32_t, MOTOR_COUNT> &requested_steps) const
    {
        std::array<int32_t, MOTOR_COUNT> limited_steps = requested_steps;

        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            int32_t new_position = current_position[i] + requested_steps[i];

            if (new_position < MIN_LIMITS[i])
            {
                limited_steps[i] = MIN_LIMITS[i] - current_position[i];
            }
            else if (new_position > MAX_LIMITS[i])
            {
                limited_steps[i] = MAX_LIMITS[i] - current_position[i];
            }
        }
        return limited_steps;
    }

    void print_position() const
    {
        ESP_LOGI(TAG, "Posición actual: [%ld, %ld, %ld]",
                 current_position[0], current_position[1], current_position[2]);
        ESP_LOGI(TAG, "Límites: Motor0[%ld,%ld] Motor1[%ld,%ld] Motor2[%ld,%ld]",
                 MIN_LIMITS[0], MAX_LIMITS[0],
                 MIN_LIMITS[1], MAX_LIMITS[1],
                 MIN_LIMITS[2], MAX_LIMITS[2]);
    }

    void reset_position()
    {
        current_position.fill(0);
        ESP_LOGI(TAG, "Posición reseteada a [0, 0, 0]");
    }
};

// Clase para controlar un motor individual
class MotorController
{
private:
    mcpwm_timer_handle_t timer = nullptr;
    mcpwm_oper_handle_t oper = nullptr;
    mcpwm_gen_handle_t gen = nullptr;
    mcpwm_cmpr_handle_t cmp = nullptr;
    uint32_t target_steps = 0;
    uint32_t step_count = 0;
    int32_t actual_steps_to_move = 0; // Pasos reales que se van a mover (puede ser limitado)
    bool active = false;
    bool direction = true; // true = positivo, false = negativo
    int motor_index;

public:
    MotorController(int index) : motor_index(index) {}

    ~MotorController()
    {
        cleanup();
    }

    // Callback estático que redirige a la instancia
    static bool IRAM_ATTR timer_callback(mcpwm_timer_handle_t timer,
                                         const mcpwm_timer_event_data_t *edata,
                                         void *user_ctx)
    {
        MotorController *motor = static_cast<MotorController *>(user_ctx);
        motor->step_count++;
        if (motor->step_count >= motor->target_steps)
        {
            mcpwm_timer_start_stop(motor->timer, MCPWM_TIMER_STOP_EMPTY);
            mcpwm_generator_set_force_level(motor->gen, 0, true);
            motor->active = false;
            return true; // Stop timer
        }
        return false;
    }

    void setup(float frequency_hz)
    {
        uint32_t period_ticks = MCPWM_RES_HZ / frequency_hz;

        // Timer individual para cada motor
        mcpwm_timer_config_t timer_cfg = {};
        timer_cfg.group_id = 0;
        timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        timer_cfg.resolution_hz = MCPWM_RES_HZ;
        timer_cfg.period_ticks = period_ticks;
        timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

        // Operador
        mcpwm_operator_config_t oper_cfg = {};
        oper_cfg.group_id = 0;
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

        // Generador
        mcpwm_generator_config_t gen_cfg = {};
        gen_cfg.gen_gpio_num = STEP_PINS[motor_index];
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &gen));

        // Comparador
        mcpwm_comparator_config_t cmp_cfg = {};
        cmp_cfg.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &cmp));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp, period_ticks / 2));

        // Generar pulso 50%
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_LOW)));

        // Registrar callback del timer
        mcpwm_timer_event_callbacks_t cbs = {};
        cbs.on_full = timer_callback;
        ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer, &cbs, this));
    }

    void cleanup()
    {
        if (timer)
        {
            mcpwm_timer_disable(timer);
            mcpwm_del_timer(timer);
            timer = nullptr;
        }

        if (gen)
        {
            mcpwm_del_generator(gen);
            gen = nullptr;
        }

        if (cmp)
        {
            mcpwm_del_comparator(cmp);
            cmp = nullptr;
        }

        if (oper)
        {
            mcpwm_del_operator(oper);
            oper = nullptr;
        }
    }

    void set_direction(bool dir)
    {
        direction = dir;
        gpio_set_level(static_cast<gpio_num_t>(DIR_PINS[motor_index]), dir ? 1 : 0);
    }

    void set_target_steps(uint32_t steps, int32_t actual_steps)
    {
        target_steps = steps;
        actual_steps_to_move = actual_steps;
        step_count = 0;
        active = steps > 0;
    }

    int32_t get_actual_steps_moved() const { return actual_steps_to_move; }

    bool is_active() const { return active; }

    void enable()
    {
        if (timer)
        {
            ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
        }
    }

    void start()
    {
        if (timer && active)
        {
            ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
        }
    }
};

// Clase principal para el sistema de motores sincronizados
class SynchronizedMotorSystem
{
private:
    std::array<MotorController, MOTOR_COUNT> motors;
    RobotPosition robot_position; // Rastreador de posición del robot

public:
    SynchronizedMotorSystem() : motors{MotorController(0), MotorController(1), MotorController(2)}
    {
        init_direction_pins();
        robot_position.print_position();
    }

    void init_direction_pins()
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            gpio_config_t dir_conf = {};
            dir_conf.pin_bit_mask = (1ULL << DIR_PINS[i]);
            dir_conf.mode = GPIO_MODE_OUTPUT;
            dir_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            dir_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            dir_conf.intr_type = GPIO_INTR_DISABLE;
            ESP_ERROR_CHECK(gpio_config(&dir_conf));
            gpio_set_level(static_cast<gpio_num_t>(DIR_PINS[i]), 0); // Inicializar en LOW
        }
        ESP_LOGI(TAG, "Pines de dirección inicializados");
    }

    void execute_synchronized_move(const std::array<int32_t, MOTOR_COUNT> &steps, float duration_sec)
    {
        ESP_LOGI(TAG, "== Iniciando movimiento sincronizado ==");

        // Verificar y ajustar movimiento según límites
        std::array<int32_t, MOTOR_COUNT> limited_steps = robot_position.get_limited_steps(steps);

        // Verificar si hubo limitaciones
        bool was_limited = false;
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            if (limited_steps[i] != steps[i])
            {
                was_limited = true;
                ESP_LOGW(TAG, "Motor %d limitado: solicitado=%ld, permitido=%ld",
                         i, steps[i], limited_steps[i]);
            }
        }

        if (was_limited)
        {
            ESP_LOGW(TAG, "Movimiento fue limitado por restricciones de posición");
        }

        // Configurar cada motor
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            bool direction = limited_steps[i] >= 0;
            uint32_t abs_steps = std::abs(limited_steps[i]);

            motors[i].set_target_steps(abs_steps, limited_steps[i]);

            if (abs_steps > 0)
            {
                motors[i].set_direction(direction);
                float freq = static_cast<float>(abs_steps) / duration_sec;
                ESP_LOGI(TAG, "Motor %d: %ld pasos (%s), %.2f Hz",
                         i, limited_steps[i], direction ? "+" : "-", freq);
                motors[i].setup(freq);
            }
            else
            {
                ESP_LOGI(TAG, "Motor %d: Sin movimiento", i);
            }
        }

        // Habilitar timers
        for (auto &motor : motors)
        {
            if (motor.is_active())
            {
                motor.enable();
            }
        }

        // Iniciar todos al mismo tiempo
        for (auto &motor : motors)
        {
            motor.start();
        }

        // Esperar a que todos terminen
        bool all_done = false;
        while (!all_done)
        {
            all_done = true;
            for (const auto &motor : motors)
            {
                if (motor.is_active())
                {
                    all_done = false;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // Limpiar recursos después del movimiento
        for (auto &motor : motors)
        {
            motor.cleanup();
        }

        // Actualizar posición del robot
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            robot_position.update_position(i, motors[i].get_actual_steps_moved());
        }

        ESP_LOGI(TAG, "== Movimiento finalizado ==");
        robot_position.print_position();
    }

    void execute_movement_sequence(const std::vector<Movement> &movements)
    {
        ESP_LOGI(TAG, "=== Iniciando secuencia de %zu movimientos ===", movements.size());

        for (size_t i = 0; i < movements.size(); i++)
        {
            const auto &movement = movements[i];
            ESP_LOGI(TAG, "Ejecutando movimiento %zu/%zu", i + 1, movements.size());
            ESP_LOGI(TAG, "Pasos: [%ld, %ld, %ld], Duración: %.2fs",
                     movement.steps[0], movement.steps[1], movement.steps[2],
                     movement.duration_sec);

            execute_synchronized_move(movement.steps, movement.duration_sec);

            if (movement.delay_after_ms > 0)
            {
                ESP_LOGI(TAG, "Pausa de %lu ms", movement.delay_after_ms);
                vTaskDelay(pdMS_TO_TICKS(movement.delay_after_ms));
            }
        }

        ESP_LOGI(TAG, "=== Secuencia completada ===");
    }

    // Métodos adicionales para gestión de posición
    void reset_position()
    {
        robot_position.reset_position();
    }

    void print_current_position() const
    {
        robot_position.print_position();
    }

    void set_position(const std::array<int32_t, MOTOR_COUNT> &position)
    {
        robot_position.current_position = position;
        ESP_LOGI(TAG, "Posición establecida manualmente a [%ld, %ld, %ld]",
                 position[0], position[1], position[2]);
    }

    std::array<int32_t, MOTOR_COUNT> get_current_position() const
    {
        return robot_position.current_position;
    }

    bool can_move_to(const std::array<int32_t, MOTOR_COUNT> &target_steps) const
    {
        return robot_position.is_move_valid(target_steps);
    }
};

// Función de tarea para el planificador
void planner_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));

    // Crear sistema de motores sincronizados
    SynchronizedMotorSystem motor_system;

    // Secuencia de movimientos usando inicialización moderna de C++
    std::vector<Movement> sequence = {
        Movement({0, 2100, 0}, 0.5f, 1000),     // Q2 Full positivo
        Movement({0, 0, 700}, 0.5f, 1000),      // Q3 Full positivo
        Movement({3200, 0, 0}, 0.5f, 1000),     // Q1 Full positivo
        Movement({-1600, 0, -350}, 1.0f, 1000), // Q1 y Q3 hacia atrás
        Movement({0, -1050, 0}, 0.5f, 1000),    // Q2 hacia atrás
        Movement({-10000, 0, 0}, 1.0f, 1000),   // Movimiento que será limitado
        Movement({10000, 0, 0}, 1.0f, 1000),   // Movimiento que será limitado
    };

    ESP_LOGI(TAG, "Iniciando sistema con límites de seguridad");
    motor_system.print_current_position();

    // Ejecutar secuencia
    motor_system.execute_movement_sequence(sequence);

    ESP_LOGI(TAG, "Secuencia completada. Posición final:");
    motor_system.print_current_position();

    // Ejemplo de verificación de movimiento
    std::array<int32_t, 3> test_move = {1000, 1000, 1000};
    if (motor_system.can_move_to(test_move))
    {
        ESP_LOGI(TAG, "El movimiento [1000, 1000, 1000] es válido");
    }
    else
    {
        ESP_LOGI(TAG, "El movimiento [1000, 1000, 1000] excede los límites");
    }

    // Mantener la tarea activa
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main(void)
{
    xTaskCreate(planner_task, "planner_cpp", 8192, nullptr, 5, nullptr);
}
