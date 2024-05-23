# STM32F303_NUCLEO_64_CAN-Bus_Normal_Arduino
Proof of concept test project for HAL-based CAN-bus communication in normal mode with a STM32F303 NUCLEO-64 board (MCP2551)and Arduino UNO (MCP2515). 
A messages with different a 11-bit ID is sent and filtered to a FIFO mailbox and then handled by a callback function.
