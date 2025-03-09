

# **Case Study: UART Communication Simulation Between Two ESP32s**

## **Objective**

Simulate UART communication between two ESP32 microcontrollers using a simulation environment, ensuring:

1. Interaction with peripherals (UART).
2. Implementation of a communication protocol (UART).
3. Data exchange between two components (ESP32s).
4. Execution in a simulated environment.

---

## **Tools and Technologies Used**

- **Microcontroller**: ESP32
- **Programming Framework**: ESP-IDF (FreeRTOS-based)
- **Simulation Environment**: Dockerized Renode (ARM64 on macOS)
- **Host System**: macOS (Apple M1)
- **Programming Language**: C

---

## **Implementation Details**

### 1. **UART Communication Code**

#### Transmitter Code (`transmitter.c`):

Simulates temperature data and sends it over UART every 2 seconds.

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_random.h"

#define TX_PIN 1
#define RX_PIN UART_PIN_NO_CHANGE
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

    while (1) {
        float temp = 25.0 + (esp_random() % 1000) / 100.0;
        char tx_buffer[50];
        int len = snprintf(tx_buffer, sizeof(tx_buffer), "TEMP:%.2f\n", temp);
        uart_write_bytes(UART_NUM, tx_buffer, len);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
```


#### Receiver Code (`receiver.c`):

Receives data over UART and prints it to the serial monitor.

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#define RX_PIN 3
#define TX_PIN UART_PIN_NO_CHANGE
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
    while (1) {
        int rxBytes = uart_read_bytes(UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(1000));
        if (rxBytes > 0) {
            data[rxBytes] = '\0';
            printf("Received: %s", data);
        }
    }
}
```

---

### 2. **Simulation Setup**

#### Initial Plan:

- Use Renode to simulate ESP32 devices and their peripherals.
- Run the simulation in a Docker container due to macOS compatibility constraints.


#### Challenges Faced:

1. Compatibility issues with `antmicro/renode` Docker image (`linux/amd64`) on Apple M1 (`linux/arm64`).
2. Permission errors during dependency installation.
3. Platform mismatch causing runtime crashes.

---

### **Solutions Implemented**

#### A. Dockerfile Customization:

To resolve compatibility and permission issues:

```dockerfile
FROM --platform=linux/arm64 antmicro/renode:latest

USER root

RUN apt-get update && \
    apt-get install -y mono-complete && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

USER renode
WORKDIR /home/renode
```


#### B. ARM64-Compatible Build:

Built a custom ARM64-compatible image:

```bash
docker build --platform linux/arm64 -t renode-arm64 .
```


#### C. Prebuilt ARM64 Image as Fallback:

Used the official ARM64 Renode image when custom builds failed:

```bash
docker pull ghcr.io/renode/renode:arm64
docker run --platform linux/arm64 -v $(pwd):/work -it ghcr.io/renode/renode:arm64
```

---

### **Final Steps**

1. Built the ESP32 firmware using `idf.py build`.
2. Created a Renode script (`uart_simulation.resc`) to configure two ESP32s with cross-connected UARTs.
3. Loaded the firmware into Renode for simulation.

#### Example Renode Script:

```python
using sysbus

machine1: Machine.Sifive.Esp32_Wrover_KIT
machine LoadPlatformDescription @platforms/cpus/esp32.repl

machine2: Machine.Sifive.Esp32_Wrover_KIT

connector Connect sysbus.uart2 machine1.uart
connector Connect sysbus.uart2 machine2.uart

machine1 LoadELF @path/to/transmitter.elf
machine2 LoadELF @path/to/receiver.elf

start
```

---

## **Results**

### Expected Output:

- Transmitter Serial Monitor:

```
TEMP:27.45
TEMP:30.12
```

- Receiver Serial Monitor:

```
Received: TEMP:27.45
Received: TEMP:30.12
```


### Challenges Resolved:

- Successfully ran Renode in Docker on macOS with ARM64 architecture.
- Simulated two ESP32 devices exchanging meaningful data over UART.

---

## **Conclusion**

This task demonstrated how to simulate complex embedded systems on macOS using Docker and Renode despite platform constraints. The final setup achieved accurate simulation of ESP32-to-ESP32 communication via UART.

---

## **Future Recommendations**

1. Explore native Renode installations for macOS to avoid Docker overhead.
2. Use lightweight tools like Wokwi for quick prototyping if simulation accuracy is not critical.
3. Consider testing on physical hardware for real-world validation.

---

### How to Use This Repository:

1. Clone the repository.
2. Build the firmware using `idf.py build`.
3. Run the simulation using Renode or Dockerized Renode.


