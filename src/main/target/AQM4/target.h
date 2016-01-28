/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#define TARGET_BOARD_IDENTIFIER "AQM4"

#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)
#define CONFIG_SERIALRX_PROVIDER SERIALRX_SPEKTRUM2048
//#define CONFIG_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD
#define CONFIG_FEATURE_RX_SERIAL
#define CONFIG_MSP_PORT 1
#define CONFIG_RX_SERIAL_PORT 2

#define USBD_PRODUCT_STRING "AeroQuad32 V2"

#define LED0 PA15               // Ready Green LED0
#define LED1 PC2                // GPS Blue LED1
#define LED2 PB15               // Debug Yellow LED2

#define BEEPER PC13             // Buzzer port

#define INVERTER PC15           // PC15 used as inverter select GPIO
#define INVERTER_USART USART2

#define MPU6000_CS_PIN        PC14
#define MPU6000_SPI_INSTANCE  SPI3

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW270_DEG

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW270_DEG


#define HMC5883_CS_PIN        PC13
#define HMC5883_SPI_INSTANCE  SPI3

//#define MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN CW180_DEG

#define MS5611_CS_PIN         PA8
#define MS5611_SPI_INSTANCE   SPI3

//#define BARO
#define USE_BARO_MS5611

//#define USE_SDCARD

//#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PA4
#define SDCARD_DETECT_EXTI_LINE             EXTI_Line4
#define SDCARD_DETECT_EXTI_PIN_SOURCE       EXTI_PinSource4
#define SDCARD_DETECT_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOA
#define SDCARD_DETECT_EXTI_IRQn             EXTI2_IRQn

#define SDCARD_SPI_INSTANCE                 SPI1
#define SDCARD_SPI_CS_PIN                   PB11

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA2_Stream3
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF3
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA2
#define SDCARD_DMA_CHANNEL                  DMA_Channel_4

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define BRUSHED_MOTORS

//#define M25P16_CS_PIN         PC15
//#define M25P16_SPI_INSTANCE   SPI3

//#define USE_FLASHFS
//#define USE_FLASH_M25P16

#define USABLE_TIMER_CHANNEL_COUNT 9

// MPU6000 interrupt
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH
#define EXTI_CALLBACK_HANDLER_COUNT 2 // MPU and Mag data ready
#define USE_EXTI
#define MPU_INT_EXTI PC3

#define USE_VCP

#define USE_USART1
#define USART1_RX_PIN PA10
#define USART1_TX_PIN PA9
#define USART1_AHB1_PERIPHERALS RCC_AHB1Periph_DMA2

#define USE_USART2
#define USART2_RX_PIN PA3
#define USART2_TX_PIN PA5

//#define USE_USART3
#define USART3_RX_PIN PB11
#define USART3_TX_PIN PB8

//#define USE_USART4
#define USART4_RX_PIN PA1
#define USART4_TX_PIN PA0

//#define USE_USART5
#define USART5_RX_PIN PD2
#define USART5_TX_PIN PD10

//#define USE_USART6
#define USART6_RX_PIN PC7
#define USART6_TX_PIN PC6

#define SERIAL_PORT_COUNT 3

#define USE_SPI

//#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PB11
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

//#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PC14
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

//#define USE_ADC
//#define BOARD_HAS_VOLTAGE_DIVIDER

#define VBAT_ADC_GPIO_PIN           PC0
#define VBAT_ADC_CHANNEL            ADC_Channel_1

#define CURRENT_METER_ADC_GPIO_PIN  PC1
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_0

// LED strip configuration using RC5 pin.
//#define LED_STRIP
#define LED_STRIP_TIMER TIM8

#define USE_LED_STRIP_ON_DMA1_CHANNEL3
#define WS2811_GPIO                     GPIOB
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOB
#define WS2811_GPIO_AF                  GPIO_AF_3
#define WS2811_PIN                      GPIO_Pin_15 // TIM8_CH3
#define WS2811_PIN_SOURCE               GPIO_PinSource15
#define WS2811_TIMER                    TIM8
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM8
#define WS2811_DMA_CHANNEL              DMA1_Channel3
#define WS2811_IRQ                      DMA1_Channel3_IRQn

#define BLACKBOX
//#define DISPLAY
#define GPS
#define GTUNE
#define SERIAL_RX
#define TELEMETRY
#define USE_SERVOS
#define USE_CLI

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PORT  GPIOA
#define BIND_PIN   Pin_3

//#define USE_SERIAL_1WIRE
#define ESC_COUNT 8
#define S1W_TX_GPIO         GPIOA
#define S1W_TX_PIN          GPIO_Pin_9
#define S1W_RX_GPIO         GPIOA
#define S1W_RX_PIN          GPIO_Pin_10

//#define DSMX_RX
#ifdef DSMX_RX
// DSMX receiver (CYRF6936)
#define CYRF_TIMER              TIM7
#define CYRF_TIMER_CLOCK        (clocks.PCLK1_Frequency * 2)
#define CYRF_TIMER_IRQ_CH       TIM7_IRQn
#define CYRF_TIMER_ISR          TIM7_IRQHandler
#define CYRF_TIMER_DBG          DBGMCU_TIM7_STOP

#define CYRF_SPI                SPI1
#define CYRF_SPI_BAUD           SPI_BaudRatePrescaler_16    // 2.125 MHz

#define CYRF_IRQ_PORT           GPIOB
#define CYRF_IRQ_PIN            GPIO_Pin_14
//#define CYRF_RST_PORT           GPIOB
//#define CYRF_RST_PIN            GPIO_Pin_6
#define CYRF_CS_PORT            GPIOB
#define CYRF_CS_PIN             GPIO_Pin_11

#define SPI_SPI1_CLOCK          RCC_APB2Periph_SPI1
#define SPI_SPI1_AF             GPIO_AF_SPI1
#define SPI_SPI1_SCK_PORT       GPIOA
#define SPI_SPI1_MISO_PORT      GPIOA
#define SPI_SPI1_MOSI_PORT      GPIOA
#define SPI_SPI1_SCK_PIN        GPIO_Pin_5
#define SPI_SPI1_MISO_PIN       GPIO_Pin_6
#define SPI_SPI1_MOSI_PIN       GPIO_Pin_7
#define SPI_SPI1_SCK_SOURCE     GPIO_PinSource5
#define SPI_SPI1_MISO_SOURCE    GPIO_PinSource6
#define SPI_SPI1_MOSI_SOURCE    GPIO_PinSource7

#define SPI_SPI1_DMA_RX         DMA2_Stream0
#define SPI_SPI1_DMA_RX_CHANNEL DMA_Channel_3
#define SPI_SPI1_DMA_RX_FLAGS   (DMA_IT_TEIF0 | DMA_IT_DMEIF0 | DMA_IT_FEIF0 | DMA_IT_TCIF0 | DMA_IT_HTIF0)
#define SPI_SPI1_DMA_RX_IRQ     DMA2_Stream0_IRQn
#define SPI_SPI1_DMA_RX_HANDLER DMA2_Stream0_IRQHandler

#define SPI_SPI1_DMA_TX         DMA2_Stream5
#define SPI_SPI1_DMA_TX_CHANNEL DMA_Channel_3
#define SPI_SPI1_DMA_TX_FLAGS   (DMA_IT_TEIF5 | DMA_IT_DMEIF5 | DMA_IT_FEIF5 | DMA_IT_TCIF5 | DMA_IT_HTIF5)
#endif

#define USE_QUATERNION

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
