#include "app_main.h"
#include "depth_parser.h"
#include "depth_transport_stm32.h"

#include <stddef.h>

static void compile_jy901s_app_api(void)
{
    UART_HandleTypeDef uart1 = {0};
    UART_HandleTypeDef uart3 = {0};
    UART_HandleTypeDef uart6 = {0};
    TIM_HandleTypeDef tim3 = {0};
    TIM_HandleTypeDef tim4 = {0};
    jy901s_imu_state_t state;
    jy901s_parser_stats_t parser_stats;
    jy901s_transport_stm32_diagnostics_t transport_diagnostics;
    depth_parser_state_t depth_state;
    depth_parser_stats_t depth_parser_stats;
    depth_transport_stm32_diagnostics_t depth_transport_diagnostics;

    app_main_init(
        &uart1,
        &uart3,
        &uart6,
        &tim3,
        &tim4,
        (GPIO_TypeDef *)0,
        0U);
    app_main_jy901s_get_state(&state);
    app_main_jy901s_get_parser_stats(&parser_stats);
    app_main_jy901s_get_transport_diagnostics(&transport_diagnostics);
    (void)app_main_jy901s_last_valid_frame_ms();
    app_main_depth_get_state(&depth_state);
    app_main_depth_get_parser_stats(&depth_parser_stats);
    app_main_depth_get_transport_diagnostics(&depth_transport_diagnostics);
}

int main(void)
{
    compile_jy901s_app_api();
    return 0;
}
