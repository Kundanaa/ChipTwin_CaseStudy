#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_random.h"

// Wokwi Virtual Pins
#define TX_PIN 1  // UART1_TX -> Connected to receiver's RX
#define RX_PIN UART_PIN_NO_CHANGE  // Not used
#define BUF_SIZE 1024
#define UART_NUM UART_NUM_1

void tx_task(void *pvParam) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);

    while(1) {
        float temp = 25.0 + (esp_random() % 1000)/100.0;
        char tx_buffer[50];
        int len = snprintf(tx_buffer, sizeof(tx_buffer), "TEMP:%.2f\n", temp);
        uart_write_bytes(UART_NUM, tx_buffer, len);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/uart.h"
// #include "esp_random.h"  // Critical fix from search [3][4]
// #include "config.h"  // Add this at the top

// // Pin definitions (MUST match Renode script)
// // #define TX_PIN 17
// // #define RX_PIN 16
// // #define BUF_SIZE 1024
// // #define UART_NUM UART_NUM_2
// #define TX_PIN 1  // Wokwi's virtual UART1 TX
// #define RX_PIN 3  // Not used for transmitter
// #define BUF_SIZE 1024
// #define UART_NUM UART_NUM_1


// void tx_task(void *pvParam)
// {
//     const uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT,
//     };

//     uart_param_config(UART_NUM, &uart_config);
//     uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);

//     while(1) {
//         float temp = 25.0 + (esp_random() % 1000)/100.0; // Requires esp_random.h [3]
//         char tx_buffer[50];
//         int len = snprintf(tx_buffer, sizeof(tx_buffer), "TEMP:%.2f\n", temp);
//         uart_write_bytes(UART_NUM, tx_buffer, len);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }

