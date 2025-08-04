#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <sstream>
#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

constexpr int MOTOR_COUNT = 3;
constexpr uint32_t MCPWM_RES_HZ = 1000000; // 1 MHz resolución
constexpr std::array<int, MOTOR_COUNT> STEP_PINS = {27, 33, 19};
constexpr std::array<int, MOTOR_COUNT> DIR_PINS = {26, 32, 21};

// Límites de movimiento para cada motor (en pasos)
constexpr std::array<int32_t, MOTOR_COUNT> MIN_LIMITS = {0, 0, 0};         // Límites mínimos
constexpr std::array<int32_t, MOTOR_COUNT> MAX_LIMITS = {3200, 2100, 700}; // Límites máximos

constexpr const char *TAG = "SYNC_TIMERS_CPP";

// Estructura para comandos
struct Command
{
    enum Type
    {
        MOVE,
        MOVE_TO,
        GET_POSITION,
        RESET_POSITION,
        SET_POSITION,
        GET_LIMITS,
        STOP,
        STATUS
    };

    Type type;
    std::array<int32_t, MOTOR_COUNT> params = {0, 0, 0};
    float duration = 1.0f;
    std::string response;
};

// Cola para comandos
static QueueHandle_t command_queue = nullptr;

// Forward declaration para evitar dependencias circulares
class SynchronizedMotorSystem;

// Clase base abstracta para interfaces de comunicación
class CommunicationInterface
{
public:
    virtual ~CommunicationInterface() = default;
    virtual void send_response(const std::string &response) = 0;
    virtual bool receive_command(Command &cmd) = 0;
    virtual void init() = 0;
    virtual const char *get_name() const = 0;
};

// Interfaz UART (Puerto Serie)
class UARTInterface : public CommunicationInterface
{
private:
    static constexpr uart_port_t UART_NUM = UART_NUM_0;
    static constexpr int BUF_SIZE = 1024;
    char rx_buffer[BUF_SIZE];

public:
    void init() override
    {
        uart_config_t uart_config = {};
        uart_config.baud_rate = 115200;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.rx_flow_ctrl_thresh = 122;

        ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

        ESP_LOGI(TAG, "UART interface initialized at 115200 baud");
    }

    void send_response(const std::string &response) override
    {
        std::string full_response = response + "\r\n";
        uart_write_bytes(UART_NUM, full_response.c_str(), full_response.length());
    }

    bool receive_command(Command &cmd) override
    {
        int len = uart_read_bytes(UART_NUM, rx_buffer, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            rx_buffer[len] = '\0';
            return parse_command(std::string(rx_buffer), cmd);
        }
        return false;
    }

    const char *get_name() const override
    {
        return "UART";
    }

private:
    bool parse_command(const std::string &input, Command &cmd)
    {
        std::istringstream iss(input);
        std::string command;
        iss >> command;

        // Convertir a mayúsculas para facilitar comparación
        for (auto &c : command)
            c = std::toupper(c);

        if (command == "MOVE")
        {
            cmd.type = Command::MOVE;
            iss >> cmd.params[0] >> cmd.params[1] >> cmd.params[2] >> cmd.duration;
            return true;
        }
        else if (command == "MOVETO")
        {
            cmd.type = Command::MOVE_TO;
            iss >> cmd.params[0] >> cmd.params[1] >> cmd.params[2] >> cmd.duration;
            return true;
        }
        else if (command == "POS" || command == "POSITION")
        {
            cmd.type = Command::GET_POSITION;
            return true;
        }
        else if (command == "RESET")
        {
            cmd.type = Command::RESET_POSITION;
            return true;
        }
        else if (command == "SETPOS")
        {
            cmd.type = Command::SET_POSITION;
            iss >> cmd.params[0] >> cmd.params[1] >> cmd.params[2];
            return true;
        }
        else if (command == "LIMITS")
        {
            cmd.type = Command::GET_LIMITS;
            return true;
        }
        else if (command == "STOP")
        {
            cmd.type = Command::STOP;
            return true;
        }
        else if (command == "STATUS")
        {
            cmd.type = Command::STATUS;
            return true;
        }

        return false;
    }
};

// Interfaz Web HTTP
class WebInterface : public CommunicationInterface
{
private:
    httpd_handle_t server = nullptr;
    std::string last_response;

    static esp_err_t command_handler(httpd_req_t *req)
    {
        WebInterface *self = static_cast<WebInterface *>(req->user_ctx);

        char buf[1000];
        int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
        if (ret <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                httpd_resp_send_408(req);
            }
            return ESP_FAIL;
        }
        buf[ret] = '\0';

        Command cmd;
        if (self->parse_web_command(std::string(buf), cmd))
        {
            xQueueSend(command_queue, &cmd, 0);
            httpd_resp_send(req, "{\"status\":\"ok\",\"message\":\"Command received\"}",
                            HTTPD_RESP_USE_STRLEN);
        }
        else
        {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid command");
        }

        return ESP_OK;
    }

    static esp_err_t status_handler(httpd_req_t *req)
    {
        WebInterface *self = static_cast<WebInterface *>(req->user_ctx);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, self->last_response.c_str(), HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    static esp_err_t control_page_handler(httpd_req_t *req)
    {
        static const char html_page[] =
            "<!DOCTYPE html>\n"
            "<html><head><title>Motor Control</title></head><body>\n"
            "<h1>Motor Control</h1>\n"
            "<div>Motor 1: <input type=\"number\" id=\"m1\" value=\"0\"></div>\n"
            "<div>Motor 2: <input type=\"number\" id=\"m2\" value=\"0\"></div>\n"
            "<div>Motor 3: <input type=\"number\" id=\"m3\" value=\"0\"></div>\n"
            "<div>Duration: <input type=\"number\" id=\"dur\" value=\"1\" step=\"0.1\">s</div>\n"
            "<div><button onclick=\"sendMove()\">Move</button></div>\n"
            "<div><button onclick=\"getPos()\">Position</button></div>\n"
            "<div id=\"status\">Ready</div>\n"
            "<script>\n"
            "function sendMove(){\n"
            "var m1=document.getElementById('m1').value;\n"
            "var m2=document.getElementById('m2').value;\n"
            "var m3=document.getElementById('m3').value;\n"
            "var dur=document.getElementById('dur').value;\n"
            "var cmd='MOVE '+m1+' '+m2+' '+m3+' '+dur;\n"
            "fetch('/command',{method:'POST',body:cmd});\n"
            "}\n"
            "function getPos(){\n"
            "fetch('/command',{method:'POST',body:'POS'});\n"
            "}\n"
            "</script>\n"
            "</body></html>";

        httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

public:
    void init() override
    {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port = 80;

        if (httpd_start(&server, &config) == ESP_OK)
        {
            // Página principal
            httpd_uri_t root_uri = {
                .uri = "/",
                .method = HTTP_GET,
                .handler = control_page_handler,
                .user_ctx = this};
            httpd_register_uri_handler(server, &root_uri);

            // Handler para comandos
            httpd_uri_t command_uri = {
                .uri = "/command",
                .method = HTTP_POST,
                .handler = command_handler,
                .user_ctx = this};
            httpd_register_uri_handler(server, &command_uri);

            // Handler para status
            httpd_uri_t status_uri = {
                .uri = "/status",
                .method = HTTP_GET,
                .handler = status_handler,
                .user_ctx = this};
            httpd_register_uri_handler(server, &status_uri);

            ESP_LOGI(TAG, "Web interface started on port 80");
        }
    }

    void send_response(const std::string &response) override
    {
        last_response = response;
        ESP_LOGI(TAG, "Web Response: %s", response.c_str());
    }

    bool receive_command(Command &cmd) override
    {
        // Para la interfaz web, los comandos llegan vía HTTP handlers
        // Este método no se usa directamente
        return false;
    }

    const char *get_name() const override
    {
        return "Web";
    }

private:
    bool parse_web_command(const std::string &input, Command &cmd)
    {
        std::istringstream iss(input);
        std::string command;
        iss >> command;

        for (auto &c : command)
            c = std::toupper(c);

        if (command == "MOVE")
        {
            cmd.type = Command::MOVE;
            iss >> cmd.params[0] >> cmd.params[1] >> cmd.params[2] >> cmd.duration;
            return true;
        }
        else if (command == "MOVETO")
        {
            cmd.type = Command::MOVE_TO;
            iss >> cmd.params[0] >> cmd.params[1] >> cmd.params[2] >> cmd.duration;
            return true;
        }
        else if (command == "POS")
        {
            cmd.type = Command::GET_POSITION;
            return true;
        }
        else if (command == "RESET")
        {
            cmd.type = Command::RESET_POSITION;
            return true;
        }
        else if (command == "LIMITS")
        {
            cmd.type = Command::GET_LIMITS;
            return true;
        }
        else if (command == "STOP")
        {
            cmd.type = Command::STOP;
            return true;
        }

        return false;
    }
};

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

// Forward declaration del callback
static bool motor_timer_callback(mcpwm_timer_handle_t timer,
                                 const mcpwm_timer_event_data_t *edata,
                                 void *user_ctx);

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

    // Método para acceso del callback
    void increment_step()
    {
        step_count++;
        if (step_count >= target_steps)
        {
            mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
            mcpwm_generator_set_force_level(gen, 0, true);
            active = false;
        }
    }

    uint32_t get_step_count() const { return step_count; }
    uint32_t get_target_steps() const { return target_steps; }

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
        cbs.on_full = motor_timer_callback;
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

// Implementación del callback global
static bool IRAM_ATTR motor_timer_callback(mcpwm_timer_handle_t timer,
                                           const mcpwm_timer_event_data_t *edata,
                                           void *user_ctx)
{
    MotorController *motor = static_cast<MotorController *>(user_ctx);
    motor->increment_step();
    return (motor->get_step_count() >= motor->get_target_steps());
}

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

        // Verificar si hay movimiento real
        bool has_movement = false;
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            if (steps[i] != 0)
            {
                has_movement = true;
                break;
            }
        }

        if (!has_movement)
        {
            ESP_LOGI(TAG, "No hay movimiento que realizar - todos los pasos son 0");
            ESP_LOGI(TAG, "== Movimiento finalizado ==");
            robot_position.print_position();
            return;
        }

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
        uint32_t timeout_counter = 0;
        const uint32_t MAX_TIMEOUT = duration_sec * 1000 + 5000; // Duración + 5 segundos de margen

        while (!all_done && timeout_counter < MAX_TIMEOUT)
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

            // Alimentar el watchdog y dar tiempo a otras tareas
            vTaskDelay(pdMS_TO_TICKS(10)); // Aumentado de 1ms a 10ms
            timeout_counter += 10;
        }

        if (timeout_counter >= MAX_TIMEOUT)
        {
            ESP_LOGW(TAG, "Timeout en movimiento - forzando finalización");
            for (auto &motor : motors)
            {
                if (motor.is_active())
                {
                    motor.cleanup();
                }
            }
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

// Procesador de comandos
class CommandProcessor
{
private:
    SynchronizedMotorSystem *motor_system;
    std::vector<std::unique_ptr<CommunicationInterface>> interfaces;

public:
    CommandProcessor(SynchronizedMotorSystem *system) : motor_system(system)
    {
        command_queue = xQueueCreate(10, sizeof(Command));
    }

    void add_interface(std::unique_ptr<CommunicationInterface> interface)
    {
        interface->init();
        interfaces.push_back(std::move(interface));
        ESP_LOGI(TAG, "Added %s interface", interfaces.back()->get_name());
    }

    void process_commands()
    {
        Command cmd;

        if (xQueueReceive(command_queue, &cmd, 0) == pdTRUE)
        {
            std::string response = execute_command(cmd);
            for (auto &interface : interfaces)
            {
                interface->send_response(response);
            }
        }

        for (auto &interface : interfaces)
        {
            if (interface->receive_command(cmd))
            {
                std::string response = execute_command(cmd);
                interface->send_response(response);
            }
        }
    }

private:
    std::string execute_command(const Command &cmd)
    {
        std::ostringstream response;

        switch (cmd.type)
        {
        case Command::MOVE:
            response << "Moving: [" << cmd.params[0] << "," << cmd.params[1]
                     << "," << cmd.params[2] << "] t:" << cmd.duration << "s";
            motor_system->execute_synchronized_move(cmd.params, cmd.duration);
            break;
        case Command::MOVE_TO:
        {
            auto current = motor_system->get_current_position();
            std::array<int32_t, MOTOR_COUNT> relative_move;
            bool has_movement = false;

            for (int i = 0; i < MOTOR_COUNT; i++)
            {
                relative_move[i] = cmd.params[i] - current[i];
                if (relative_move[i] != 0)
                {
                    has_movement = true;
                }
            }

            if (!has_movement)
            {
                response << "MoveTo: [" << cmd.params[0] << "," << cmd.params[1] << "," << cmd.params[2] << "] - Ya en posición destino";
            }
            else
            {
                response << "MoveTo: [" << cmd.params[0] << "," << cmd.params[1] << "," << cmd.params[2] << "] (rel: [" << relative_move[0] << "," << relative_move[1] << "," << relative_move[2] << "])";
                motor_system->execute_synchronized_move(relative_move, cmd.duration);
            }
            break;
        }
        case Command::GET_POSITION:
        {
            auto pos = motor_system->get_current_position();
            response << "Position: [" << pos[0] << "," << pos[1] << "," << pos[2] << "]";
            break;
        }
        case Command::RESET_POSITION:
            motor_system->reset_position();
            response << "Position reset to [0,0,0]";
            break;
        case Command::SET_POSITION:
            motor_system->set_position(cmd.params);
            response << "Position set to [" << cmd.params[0] << "," << cmd.params[1] << "," << cmd.params[2] << "]";
            break;
        case Command::GET_LIMITS:
            response << "Limits: M0[" << MIN_LIMITS[0] << "," << MAX_LIMITS[0] << "] M1[" << MIN_LIMITS[1] << "," << MAX_LIMITS[1] << "] M2[" << MIN_LIMITS[2] << "," << MAX_LIMITS[2] << "]";
            break;
        case Command::STOP:
            response << "STOP command received";
            break;
        case Command::STATUS:
        {
            auto pos = motor_system->get_current_position();
            response << "Status: OK, Pos: [" << pos[0] << "," << pos[1] << "," << pos[2] << "]";
            break;
        }
        default:
            response << "Unknown command";
            break;
        }

        return response.str();
    }
};

// Función de tarea para el procesamiento de comandos
void command_task(void *arg)
{
    CommandProcessor *processor = static_cast<CommandProcessor *>(arg);

    while (true)
    {
        processor->process_commands();
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
    }
}

// Función de tarea para el planificador
void planner_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar inicialización

    // Crear sistema de motores sincronizados
    SynchronizedMotorSystem motor_system;

    // Crear procesador de comandos
    CommandProcessor processor(&motor_system);

    // Agregar interfaces de comunicación
    processor.add_interface(std::make_unique<UARTInterface>());
    // TODO: Configurar WiFi antes de habilitar interfaz Web
    // processor.add_interface(std::make_unique<WebInterface>());

    // Crear tarea para procesamiento de comandos
    xTaskCreate(command_task, "commands", 8192, &processor, 5, nullptr);

    ESP_LOGI(TAG, "=== Robot Control System Ready ===");
    ESP_LOGI(TAG, "Available commands:");
    ESP_LOGI(TAG, "  MOVE x y z [duration] - Move relative steps");
    ESP_LOGI(TAG, "  MOVETO x y z [duration] - Move to absolute position");
    ESP_LOGI(TAG, "  POS - Get current position");
    ESP_LOGI(TAG, "  RESET - Reset position to [0,0,0]");
    ESP_LOGI(TAG, "  SETPOS x y z - Set current position");
    ESP_LOGI(TAG, "  LIMITS - Get motor limits");
    ESP_LOGI(TAG, "  STATUS - Get system status");
    ESP_LOGI(TAG, "  STOP - Emergency stop");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Interfaces available:");
    ESP_LOGI(TAG, "  - UART: 115200 baud on USB port");
    ESP_LOGI(TAG, "  - Web: Disabled (configure WiFi to enable)");

    motor_system.print_current_position();

    // Mantener la tarea activa - el sistema ahora responde a comandos
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    // Inicializar NVS para WiFi (si se usa)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xTaskCreate(planner_task, "planner_cpp", 16384, nullptr, 5, nullptr);
}
