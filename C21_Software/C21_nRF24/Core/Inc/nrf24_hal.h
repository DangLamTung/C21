#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H


// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions


// Peripheral libraries

#include "stm32f1xx_hal.h"
//#include "main.h"

// SPI port peripheral
extern SPI_HandleTypeDef hspi2;
//#define nRF24_SPI_PORT             hspi1

// nRF24 GPIO peripherals
#define nRF24_GPIO_PERIPHERALS     RCC_APB2ENR_IOPBEN

// CE (chip enable) pin (PC4)
#define nRF24_CE_PORT              GPIOA
#define nRF24_CE_PIN               GPIO_PIN_8
#define nRF24_CE_L                 HAL_GPIO_WritePin(nRF24_CE_PORT, nRF24_CE_PIN,GPIO_PIN_RESET)
#define nRF24_CE_H                 HAL_GPIO_WritePin(nRF24_CE_PORT, nRF24_CE_PIN,GPIO_PIN_SET)

// CSN (chip select negative) pin (PC5)
#define nRF24_CSN_PORT             GPIOB
#define nRF24_CSN_PIN              GPIO_PIN_12
#define nRF24_CSN_L                HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN,GPIO_PIN_RESET)
#define nRF24_CSN_H                HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN,GPIO_PIN_SET)

// IRQ pin (PA4)
#define nRF24_IRQ_PORT             GPIOA
#define nRF24_IRQ_PIN              GPIO_PIN_4


// Macros for the RX on/off
#define nRF24_RX_ON                nRF24_CE_H
#define nRF24_RX_OFF               nRF24_CE_L


// Function prototypes
void nRF24_GPIO_Init(void);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
