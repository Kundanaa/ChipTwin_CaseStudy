#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "tasks.h"

#define UART_NUM UART_NUM_2
#define BUF_SIZE 1024
#define TX_PIN 17
#define RX_PIN 16

// Declare prototypes
void tx_task(void *pvParam);
void rx_task(void *pvParam);

void app_main()
{
    // Tasks created in transmitter.c and receiver.c
    xTaskCreate(tx_task, "uart_tx", 4096, NULL, 5, NULL);
    xTaskCreate(rx_task, "uart_rx", 4096, NULL, 5, NULL);
}
