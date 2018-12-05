#ifndef NRF24_H_
#define NRF24_H_

#include "stm32f1xx_hal.h"
#include <string.h>

//====================================

#define CS_ON			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define CS_OFF		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

#define CE_RESET	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define CE_SET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)

//#define IRQ				HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)

#define LED_GREEN_ON			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
#define LED_GREEN_OFF  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)

#define LED_TOG   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14)

//------------------------------------------------

#define ACTIVATE 0x50 //
#define ACTIVATE_NO_ACK 0xB0 // Отправка без подтверждения
#define RD_RX_PLOAD 0x61 // Define RX payload register address
#define WR_TX_PLOAD 0xA0 // Define TX payload register address
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
//------------------------------------------------
#define CONFIG 0x00 //'Config' register address
#define EN_AA 0x01 //'Enable Auto Acknowledgment' register address
#define EN_RXADDR 0x02 //'Enabled RX addresses' register address
#define SETUP_AW 0x03 //'Setup address width' register address
#define SETUP_RETR 0x04 //'Setup Auto. Retrans' register address
#define RF_CH 0x05 //'RF channel' register address
#define RF_SETUP 0x06 //'RF setup' register address
#define STATUS 0x07 //'Status' register address
#define OBSERVE_TX 0x08 //'Transmit observe' register
#define RX_ADDR_P0 0x0A //'RX address pipe0' register address
#define RX_ADDR_P1 0x0B //'RX address pipe1' register address
#define TX_ADDR 0x10 //'TX address' register address
#define RX_PW_P0 0x11 //'RX payload width, pipe0' register address
#define RX_PW_P1 0x12 //'RX payload width, pipe1' register address
#define FIFO_STATUS 0x17 //'FIFO Status Register' register address
#define DYNPD 0x1C
#define FEATURE 0x1D
//------------------------------------------------
#define PRIM_RX 0x00 //RX/TX control (1: PRX, 0: PTX)
#define PWR_UP 0x01 //1: POWER UP, 0:POWER DOWN
#define RX_DR 0x40 //Data Ready RX FIFO interrupt
#define TX_DS 0x20 //Data Sent TX FIFO interrupt
#define MAX_RT 0x10 //Maximum number of TX retransmits interrupt
//------------------------------------------------
#define W_REGISTER 0x20 //запись в регистр
//------------------------------------------------
#define TX_ADR_WIDTH 3
#define TX_PLOAD_WIDTH 5
//--------------------------------------------------

void NRF24_ini(void);
uint8_t NRF24_ReadReg(uint8_t addr);
void NRF24_WriteReg(uint8_t addr, uint8_t dt);
void NRF24_ToggleFeatures(void);
void NRF24_Read_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes);
void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes);
void NRF24_FlushTX(void);
void NRF24_FlushRX(void);
void NRF24L01_RX_Mode(void);
void NRF24L01_TX_Mode(uint8_t *pBuf);
void NRF24_Transmit(uint8_t addr,uint8_t *pBuf,uint8_t bytes);
uint8_t NRF24L01_Send(uint8_t *pBuf);
void NRF24L01_Receive(void);
void IRQ_Callback(void);
void TIM1_Callback(void);


#endif // NRF24_H_