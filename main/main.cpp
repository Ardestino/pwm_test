#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

extern "C" int app_main(int argc, char const *argv[])
{
    const static char *TAG = "main";
    const uint32_t TIMER_RESOLUTION_HZ = 1000000; // 1MHz, 1us per tick
    const uint32_t TIMER_PERIOD = 1000;           // 1000 ticks,
    const uint32_t TIMER_GROUP = 0;

    ESP_LOGI(TAG, "Crear el timer");
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = TIMER_GROUP;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = TIMER_RESOLUTION_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = TIMER_PERIOD;
    timer_config.intr_priority = 0;

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    ESP_LOGI(TAG,"Crear los operadores y conectarlos al mismo timer");
    mcpwm_oper_handle_t operators[3];
    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = TIMER_GROUP;

    for (int i = 0; i < 3; ++i)
    {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
    }

    ESP_LOGI(TAG, "Create comparators");
    mcpwm_cmpr_handle_t comparators[3];
    mcpwm_comparator_config_t compare_config = {};
    compare_config.flags.update_cmp_on_tez = true;
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i]));
        // init compare for each comparator
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], 200));
    }




    return 0;
}
