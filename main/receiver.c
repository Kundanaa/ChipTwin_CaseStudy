#include <stdio.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Wokwi Virtual Pins
#define RX_PIN 3  // UART1_RX <- Connected to transmitter's TX
#define TX_PIN UART_PIN_NO_CHANGE  // Not used
#define BUF_SIZE 1024
#define UART_NUM UART_NUM_1

void rx_task(void *pvParam) {
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

    uint8_t *data = malloc(BUF_SIZE);
    if (!data) {
        printf("Memory allocation failed!\n");
        vTaskDelete(NULL);
    }

    while(1) {
        int rxBytes = uart_read_bytes(UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(1000));
        if(rxBytes > 0) {
            data[rxBytes] = '\0';
            printf("Received: %s", data);
        }
    }
    
    free(data);
    vTaskDelete(NULL);
}


// #include <stdio.h>
// #include "driver/uart.h"
// #define RX_PIN 1  // Wokwi's virtual UART1 RX
// #define TX_PIN 3  // Not used for receiver
// #define BUF_SIZE 1024
// #define UART_NUM UART_NUM_1

// #include "config.h"  // Add this at the top


// void rx_task(void *pvParam)
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

//     uint8_t *data = (uint8_t*) malloc(BUF_SIZE);
//     while(1) {
//         const int rxBytes = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, pdMS_TO_TICKS(1000));
//         if(rxBytes > 0) {
//             data[rxBytes] = 0; // Null-terminate
//             printf("Received: %s\n", data);
//         }
//     }
//     free(data);
// }
