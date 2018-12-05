# stm32f103_nrf24_tx



1. Generate project in CubeMx for Keil 5.

2. Add new existing files to group "Application/User" -> NRF24.c, New_CalibriLight14.c, ssd1306.

3. Option for Target "F103_nRF24_TX" -> Utitlites -> Settings -> check "Reset and Run"

Draft radio transmitter for RC cars.
This project uses a radio module NRF24L01 +, an OLED display 128x64 on the SSD1306, an STM32F103C8Tx processor, an encoder with a button.
Variable resistors on the axis of the accelerator and turn 4.7 kOm linear.
Receiver firmware for RC cars is here -> https://github.com/Deman75/stm32f103_nrf24l01