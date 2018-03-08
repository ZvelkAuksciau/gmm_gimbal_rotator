/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Board identifier.
 */
//#define BOARD_OLIMEX_STM32_P103
#define BOARD_NAME              "Gimbal_CTRL_Rev1"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            16000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F103xB

/*
 * IO pins assignments.
 */
#define GPIOB_LED_RED 1
#define GPIOB_LED_GREEN 2
#define GPIOA_SPI1NSS 4

#define GPIOB_IN1 4
#define GPIOB_IN2 5
#define GPIOB_IN3 0
#define GPIOB_EN1 10
#define GPIOB_EN2 11
#define GPIOB_EN3 12
#define GPIOC_RESET 14
/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything input with pull-up except:
 * PA4 - SPI1 SEL
 * PA5 - SPI1 SCK
 * PA6 - SPI1 MISO
 * PA7 - SPI1 MOSI
 * PA9  - Digital input with Pull up (USART1_TX).
 * PA10 - Alternate Push Pull (USART1_RX).
 * PA15 - Digital input with pull up (ENC_PWM).
 */
#define VAL_GPIOACRL            0x99918888      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x888888B8      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup.
 * Everything input with pull-up except:
 * PB1 PB2  - Push Pull output     (LED).
 * PB6 - Open drain alternative (I2C1_SCL) DO NOT USE. See errata
 * PB7 - Open drain alternative (I2C1_SDA) DO NOT USE. See errata
 * PB0 PB4 PB5 - Alternate Push Pull output motor driver.
 * PB10 PB11 PB12 - Push Pull output motor driver.
 * PB10 - Alternate Push pull  (USART3_TX).
 * PB11 - Input with pull up   (USART3_RX).
 */
#define VAL_GPIOBCRL            0x88BB822B      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x888222B8      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFF0CC0

/*
 * Port C setup.
 * Everything input with pull-up except:
 * PC14 reset
 */
#define VAL_GPIOCCRL            0x88888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x82888888      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 * PD0  - Normal input (XTAL).
 * PD1  - Normal input (XTAL).
 */
#define VAL_GPIODCRL            0x88888844      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

/*
 * USB bus activation macro, required by the USB driver.
 */
#define usb_lld_connect_bus(usbp) (void)//palClearPad(GPIOC, GPIOC_USB_DISC)

/*
 * USB bus de-activation macro, required by the USB driver.
 */
#define usb_lld_disconnect_bus(usbp) (void)//palSetPad(GPIOC, GPIOC_USB_DISC)

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
