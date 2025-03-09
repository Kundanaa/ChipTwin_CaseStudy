idf_component_register(
    SRCS 
    "transmitter.c"
    "receiver.c"
    "uart_comm_main.c"
    INCLUDE_DIRS 
    "include"  # Add this line
    REQUIRES 
    freertos
    driver
    esp_hw_support  # Required for esp_random [3]
)
