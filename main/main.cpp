#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <sstream>
#include <memory>
#include <cstring>
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
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "config.h"

constexpr int MOTOR_COUNT = 3;
constexpr uint32_t MCPWM_RES_HZ = 1000000; // 1 MHz resolución
constexpr std::array<int, MOTOR_COUNT> STEP_PINS = {27, 33, 19};
constexpr std::array<int, MOTOR_COUNT> DIR_PINS = {26, 32, 21};
constexpr std::array<int, MOTOR_COUNT> ENABLE_PINS = {14, 25, 18}; // Pines de habilitación para cada motor
constexpr int SPINDLE_PIN = 13; // Pin para controlar el spindle

// Límites de movimiento para cada motor (en pasos)
constexpr std::array<int32_t, MOTOR_COUNT> MIN_LIMITS = {-1000000, -1000000, -1000000};         // Límites mínimos
constexpr std::array<int32_t, MOTOR_COUNT> MAX_LIMITS = {100000, 100000, 100000}; // Límites máximos

constexpr const char *TAG = "SYNC_TIMERS_CPP";

static int s_retry_num = 0;

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
        STATUS,
        ENABLE_MOTORS,
        DISABLE_MOTORS,
        SPINDLE_ON,
        SPINDLE_OFF,
        RESTART
    };

    Type type = MOVE;
    std::array<int32_t, MOTOR_COUNT> params = {0, 0, 0};
    float duration = 1.0f;

    // Usar constructores y operadores por defecto del compilador
    Command() = default;
    Command(const Command &) = default;
    Command &operator=(const Command &) = default;
};

// Cola para comandos
static QueueHandle_t command_queue = nullptr;

// Event handler para WiFi
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < WIFI_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
    }
}

void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

// Función para inicializar SPIFFS
esp_err_t init_spiffs(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "www",
        .max_files = 5,
        .format_if_mount_failed = false
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info("www", &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS: %d kB total, %d kB used", total / 1024, used / 1024);
    }

    return ESP_OK;
}

// Función para obtener el tipo MIME basado en la extensión del archivo
const char* get_mime_type(const char* filename) {
    if (strstr(filename, ".html")) return "text/html";
    if (strstr(filename, ".js")) return "application/javascript";
    if (strstr(filename, ".css")) return "text/css";
    if (strstr(filename, ".png")) return "image/png";
    if (strstr(filename, ".jpg") || strstr(filename, ".jpeg")) return "image/jpeg";
    if (strstr(filename, ".ico")) return "image/x-icon";
    if (strstr(filename, ".svg")) return "image/svg+xml";
    return "text/plain";
}

// Función para servir archivos estáticos desde SPIFFS
esp_err_t serve_static_file(httpd_req_t *req, const char* filepath) {
    // Validación básica de seguridad para evitar path traversal
    if (strstr(filepath, "..") != NULL) {
        ESP_LOGW(TAG, "Path traversal attempt blocked: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    FILE* file = fopen(filepath, "r");
    if (!file) {
        ESP_LOGW(TAG, "File not found: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Obtener tamaño del archivo
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (file_size <= 0) {
        ESP_LOGE(TAG, "Invalid file size: %ld for file: %s", file_size, filepath);
        fclose(file);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Verificar que el archivo no sea demasiado grande (limite de 1MB)
    if (file_size > 1024 * 1024) {
        ESP_LOGE(TAG, "File too large: %ld bytes for file: %s", file_size, filepath);
        fclose(file);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Establecer tipo de contenido
    const char* mime_type = get_mime_type(filepath);
    httpd_resp_set_type(req, mime_type);
    
    // Añadir headers para UTF-8 si es HTML
    if (strstr(mime_type, "text/html")) {
        httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
    }
    
    // Añadir header de cache control
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=3600");

    // Leer y enviar archivo en chunks
    char buffer[1024];
    size_t bytes_read;
    esp_err_t send_result = ESP_OK;
    
    while ((bytes_read = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        if (httpd_resp_send_chunk(req, buffer, bytes_read) != ESP_OK) {
            ESP_LOGE(TAG, "Error sending file chunk for: %s", filepath);
            send_result = ESP_FAIL;
            break;
        }
    }
    
    // Finalizar respuesta solo si no hubo errores
    if (send_result == ESP_OK) {
        httpd_resp_send_chunk(req, NULL, 0);
        ESP_LOGI(TAG, "Successfully served file: %s (%ld bytes)", filepath, file_size);
    }
    
    fclose(file);
    return send_result;
}

// Forward declaration para evitar dependencias circulares
class SynchronizedMotorSystem;

// Clase utilitaria para parsing de comandos
class CommandParser
{
public:
    static bool parse_command(const std::string &input, Command &cmd)
    {
        // Limpiar el comando
        cmd = Command(); // Reset al estado por defecto

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
        else if (command == "ENABLE")
        {
            cmd.type = Command::ENABLE_MOTORS;
            return true;
        }
        else if (command == "DISABLE")
        {
            cmd.type = Command::DISABLE_MOTORS;
            return true;
        }
        else if (command == "SPINDLE_ON" || command == "SPINDLEON")
        {
            cmd.type = Command::SPINDLE_ON;
            return true;
        }
        else if (command == "SPINDLE_OFF" || command == "SPINDLEOFF")
        {
            cmd.type = Command::SPINDLE_OFF;
            return true;
        }
        else if (command == "RESTART" || command == "REBOOT")
        {
            cmd.type = Command::RESTART;
            return true;
        }

        return false;
    }
};

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
            return CommandParser::parse_command(std::string(rx_buffer), cmd);
        }
        return false;
    }

    const char *get_name() const override
    {
        return "UART";
    }
};

// Interfaz Web HTTP
class WebInterface : public CommunicationInterface
{
private:
    httpd_handle_t server = nullptr;
    char last_response[512] = "System Ready"; // Buffer estático para evitar problemas de memoria

    static esp_err_t command_handler(httpd_req_t *req)
    {
        char buf[1000];
        int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
        if (ret <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                ESP_LOGW(TAG, "HTTP timeout en command_handler");
                httpd_resp_send_408(req);
            }
            else
            {
                ESP_LOGW(TAG, "Error HTTP en command_handler: %d", ret);
                httpd_resp_send_500(req);
            }
            return ESP_FAIL;
        }
        buf[ret] = '\0';

        ESP_LOGI(TAG, "HTTP Command received: %s", buf);

        Command cmd;
        if (CommandParser::parse_command(std::string(buf), cmd))
        {
            xQueueSend(command_queue, &cmd, 0);

            // Crear respuesta JSON usando buffer estático
            char json_response[512];
            snprintf(json_response, sizeof(json_response),
                     "{\"status\":\"ok\",\"message\":\"Command sent: %.100s\"}", buf);

            // Configurar headers para CORS y evitar problemas de cache
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
            httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
        }
        else
        {
            ESP_LOGW(TAG, "Invalid command: %s", buf);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid command");
        }

        return ESP_OK;
    }

    static esp_err_t status_handler(httpd_req_t *req)
    {
        WebInterface *self = static_cast<WebInterface *>(req->user_ctx);

        // Crear respuesta JSON usando buffer estático
        char json_response[512];
        snprintf(json_response, sizeof(json_response),
                 "{\"status\":\"ok\",\"last_response\":\"%.300s\"}", self->last_response);

        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    static esp_err_t control_page_handler(httpd_req_t *req)
    {
        // Servir index.html desde SPIFFS
        return serve_static_file(req, "/spiffs/index.html");
    }

    static esp_err_t static_file_handler(httpd_req_t *req)
    {
        // Construir ruta del archivo basada en la URI
        char filepath[512]; // Aumentado a 512 bytes para evitar warnings
        
        // Validar que la URI no sea demasiado larga
        size_t uri_len = strlen(req->uri);
        if (uri_len > 500) { // Dejar espacio para "/spiffs" (7 chars) + null terminator
            ESP_LOGW(TAG, "URI too long (%zu chars): %.50s...", uri_len, req->uri);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        
        // Usar snprintf de forma segura
        int result = snprintf(filepath, sizeof(filepath), "/spiffs%s", req->uri);
        if (result >= (int)sizeof(filepath) || result < 0) {
            ESP_LOGW(TAG, "Filepath too long for URI: %s", req->uri);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        
        ESP_LOGI(TAG, "Serving static file: %s", filepath);
        return serve_static_file(req, filepath);
    }

public:
    void init() override
    {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port = 80;
        config.max_open_sockets = 7;
        config.max_uri_handlers = 16; // Aumentado para archivos estáticos
        config.max_resp_headers = 8;
        config.backlog_conn = 5;
        config.lru_purge_enable = true;
        config.recv_wait_timeout = 10; // 10 segundos timeout
        config.send_wait_timeout = 10; // 10 segundos timeout

        if (httpd_start(&server, &config) == ESP_OK)
        {
            // API REST para comandos (debe registrarse antes del handler genérico)
            httpd_uri_t command_uri = {
                .uri = "/api/command",
                .method = HTTP_POST,
                .handler = command_handler,
                .user_ctx = this};
            httpd_register_uri_handler(server, &command_uri);

            // API REST para status (debe registrarse antes del handler genérico)
            httpd_uri_t status_uri = {
                .uri = "/api/status",
                .method = HTTP_GET,
                .handler = status_handler,
                .user_ctx = this};
            httpd_register_uri_handler(server, &status_uri);

            // Página principal desde SPIFFS
            httpd_uri_t root_uri = {
                .uri = "/",
                .method = HTTP_GET,
                .handler = control_page_handler,
                .user_ctx = this};
            httpd_register_uri_handler(server, &root_uri);

            // Página index.html desde SPIFFS
            httpd_uri_t index_uri = {
                .uri = "/index.html",
                .method = HTTP_GET,
                .handler = control_page_handler,
                .user_ctx = this};
            httpd_register_uri_handler(server, &index_uri);

            // Handler genérico para archivos estáticos (debe registrarse al final)
            // Este handler captura todo lo que no matchee con los anteriores
            httpd_uri_t static_files_uri = {
                .uri = "/*", // Wildcard para cualquier archivo
                .method = HTTP_GET,
                .handler = static_file_handler,
                .user_ctx = this};
            httpd_register_uri_handler(server, &static_files_uri);

            ESP_LOGI(TAG, "Web interface started on port 80");
            ESP_LOGI(TAG, "Serving static files from SPIFFS partition 'www'");
            ESP_LOGI(TAG, "Access control panel at: http://[ESP32_IP]/");
        }
        else
        {
            ESP_LOGE(TAG, "Error starting HTTP server");
        }
    }

    void send_response(const std::string &response) override
    {
        // Copiar la respuesta al buffer estático
        strncpy(last_response, response.c_str(), sizeof(last_response) - 1);
        last_response[sizeof(last_response) - 1] = '\0'; // Asegurar terminación null
        ESP_LOGI(TAG, "Web Response: %s", last_response);
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
        // Log para debugging
        ESP_LOGD(TAG, "Motor %d: Setting direction pin %d to %s (logic: %s)",
                 motor_index, DIR_PINS[motor_index], dir ? "HIGH" : "LOW", dir ? "FORWARD" : "REVERSE");
        gpio_set_level(static_cast<gpio_num_t>(DIR_PINS[motor_index]), dir ? 1 : 0);
    }

    void enable_motor(bool enable = true)
    {
        // LOW para habilitar el motor (lógica invertida típica de drivers stepper)
        gpio_set_level(static_cast<gpio_num_t>(ENABLE_PINS[motor_index]), enable ? 0 : 1);
        ESP_LOGI(TAG, "Motor %d: %s (enable pin %d set to %s)",
                 motor_index, enable ? "ENABLED" : "DISABLED", 
                 ENABLE_PINS[motor_index], enable ? "LOW" : "HIGH");
    }

    void disable_motor()
    {
        enable_motor(false);
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

    // Función helper para configurar pines GPIO
    void configure_gpio_pin(int pin, gpio_mode_t mode, int initial_level)
    {
        gpio_config_t gpio_conf = {};
        gpio_conf.pin_bit_mask = (1ULL << pin);
        gpio_conf.mode = mode;
        gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_conf.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&gpio_conf));
        gpio_set_level(static_cast<gpio_num_t>(pin), initial_level);
    }

    // Función helper para esperar a que todos los motores terminen
    bool wait_for_motors_completion(float duration_sec)
    {
        const uint32_t MAX_TIMEOUT = duration_sec * 1000 + 5000; // Duración + 5 segundos de margen
        uint32_t timeout_counter = 0;

        while (timeout_counter < MAX_TIMEOUT)
        {
            // Verificar si todos los motores han terminado
            bool all_done = true;
            for (const auto &motor : motors)
            {
                if (motor.is_active())
                {
                    all_done = false;
                    break;
                }
            }

            if (all_done)
            {
                return true; // Éxito: todos los motores terminaron
            }

            // Alimentar el watchdog y dar tiempo a otras tareas
            vTaskDelay(pdMS_TO_TICKS(10));
            timeout_counter += 10;
        }

        // Timeout alcanzado
        ESP_LOGW(TAG, "Timeout en movimiento - forzando finalización");
        for (auto &motor : motors)
        {
            if (motor.is_active())
            {
                motor.cleanup();
            }
        }
        return false; // Timeout
    }

public:
    SynchronizedMotorSystem() : motors{MotorController(0), MotorController(1), MotorController(2)}
    {
        init_gpio_pins();
        robot_position.print_position();
    }

    void init_gpio_pins()
    {
        // Configurar pines de dirección (inicializar en LOW)
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            configure_gpio_pin(DIR_PINS[i], GPIO_MODE_OUTPUT, 0);
        }

        // Configurar pines de habilitación (inicializar deshabilitado - HIGH)
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            configure_gpio_pin(ENABLE_PINS[i], GPIO_MODE_OUTPUT, 1);
        }
        
        // Configurar pin del spindle (inicializar apagado - LOW)
        configure_gpio_pin(SPINDLE_PIN, GPIO_MODE_OUTPUT, 0);
        
        ESP_LOGI(TAG, "Pines de dirección, habilitación y spindle inicializados");
    }

    void execute_synchronized_move(const std::array<int32_t, MOTOR_COUNT> &steps, float duration_sec)
    {
        ESP_LOGI(TAG, "== Iniciando movimiento sincronizado ==");
        ESP_LOGI(TAG, "Pasos solicitados: [%ld, %ld, %ld], duración: %.2fs",
                 steps[0], steps[1], steps[2], duration_sec);

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

        // Mostrar posición actual antes del movimiento
        ESP_LOGI(TAG, "Posición antes del movimiento: [%ld, %ld, %ld]",
                 robot_position.current_position[0], robot_position.current_position[1], robot_position.current_position[2]);

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

                // Debug mejorado para verificar dirección
                ESP_LOGI(TAG, "Motor %d: %ld pasos (%s), %.2f Hz, dir_pin=%s, ENABLED",
                         i, limited_steps[i], direction ? "FORWARD(+)" : "REVERSE(-)", freq,
                         direction ? "HIGH" : "LOW");

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

        // Esperar a que todos terminen con timeout
        wait_for_motors_completion(duration_sec);

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

    void enable_all_motors()
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            motors[i].enable_motor(true);
        }
        ESP_LOGI(TAG, "Todos los motores habilitados");
    }

    void disable_all_motors()
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            motors[i].disable_motor();
        }
        ESP_LOGI(TAG, "Todos los motores deshabilitados");
    }

    void enable_motor(int motor_index, bool enable = true)
    {
        if (motor_index >= 0 && motor_index < MOTOR_COUNT)
        {
            motors[motor_index].enable_motor(enable);
        }
        else
        {
            ESP_LOGW(TAG, "Índice de motor inválido: %d", motor_index);
        }
    }

    void spindle_on()
    {
        gpio_set_level(static_cast<gpio_num_t>(SPINDLE_PIN), 1);
        ESP_LOGI(TAG, "Spindle encendido (pin %d = HIGH)", SPINDLE_PIN);
    }

    void spindle_off()
    {
        gpio_set_level(static_cast<gpio_num_t>(SPINDLE_PIN), 0);
        ESP_LOGI(TAG, "Spindle apagado (pin %d = LOW)", SPINDLE_PIN);
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

        // Procesar comandos de la cola
        if (xQueueReceive(command_queue, &cmd, 0) == pdTRUE)
        {
            std::string response = execute_command(cmd);
            for (auto &interface : interfaces)
            {
                interface->send_response(response);
            }
        }

        // Procesar comandos directos de interfaces (principalmente UART)
        for (auto &interface : interfaces)
        {
            Command direct_cmd;
            if (interface->receive_command(direct_cmd))
            {
                std::string response = execute_command(direct_cmd);
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
        case Command::ENABLE_MOTORS:
            motor_system->enable_all_motors();
            response << "All motors enabled";
            break;
        case Command::DISABLE_MOTORS:
            motor_system->disable_all_motors();
            response << "All motors disabled";
            break;
        case Command::SPINDLE_ON:
            motor_system->spindle_on();
            response << "Spindle ON";
            break;
        case Command::SPINDLE_OFF:
            motor_system->spindle_off();
            response << "Spindle OFF";
            break;
        case Command::RESTART:
            response << "Restarting ESP32...";
            // Esperar un momento para enviar la respuesta antes de reiniciar
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
            break;
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

    // Inicializar SPIFFS
    ESP_LOGI(TAG, "Initializing SPIFFS filesystem...");
    esp_err_t ret = init_spiffs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPIFFS, using embedded HTML as fallback");
    } else {
        ESP_LOGI(TAG, "SPIFFS initialized successfully");
    }

    // Crear sistema de motores sincronizados
    SynchronizedMotorSystem motor_system;

    // Crear procesador de comandos
    CommandProcessor processor(&motor_system);

    // Agregar interfaces de comunicación
    processor.add_interface(std::make_unique<UARTInterface>());

    // Inicializar WiFi y agregar interfaz Web
    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(3000)); // Esperar conexión WiFi
    processor.add_interface(std::make_unique<WebInterface>());

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
    ESP_LOGI(TAG, "  ENABLE - Enable all motors");
    ESP_LOGI(TAG, "  DISABLE - Disable all motors");
    ESP_LOGI(TAG, "  SPINDLE_ON - Turn spindle on");
    ESP_LOGI(TAG, "  SPINDLE_OFF - Turn spindle off");
    ESP_LOGI(TAG, "  RESTART - Restart ESP32");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Interfaces available:");
    ESP_LOGI(TAG, "  - UART: 115200 baud on USB port");
    ESP_LOGI(TAG, "  - Web: http://[ESP32_IP]/ (serving from SPIFFS)");

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
