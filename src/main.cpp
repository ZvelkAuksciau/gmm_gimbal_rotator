#include <ch.h>
#include <hal.h>
#include <math.h>

#include "node.hpp"

static const GPTConfig gpt4cfg = {
  100000, // 1 MHz timer clock.
  NULL, // No callback
  0, 0
};

/*
 * standard 9600 baud serial config.
 */
static const SerialConfig serialCfg = {
  9600,
  0,
  0,
  0
};

/*
 * Low speed SPI configuration (281.250kHz, CPHA=1, CPOL=0, MSb first).
 */
static const SPIConfig ls_spicfg = {
  NULL,
  GPIOA,
  GPIOA_SPI1NSS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPHA,
  0
};

static const PWMConfig pwm_cfg = {
  72000000,                         // 72 MHz PWM clock frequency
  1500,                             // 24 kHz PWM frequency
  NULL,                             // No Callback
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

static THD_WORKING_AREA(waThread1, 128);
void Thread1(void) {
  chRegSetThreadName("blinker");

  while(1) {
    palClearPad(GPIOB, GPIOB_LED_RED);
    palSetPad(GPIOB, GPIOB_LED_GREEN);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOB, GPIOB_LED_RED);
    palClearPad(GPIOB, GPIOB_LED_GREEN);
    chThdSleepMilliseconds(500);
  }
}

static THD_WORKING_AREA(waThread2, 128);
void Thread2(void) {
  chRegSetThreadName("rotary_position_sensor");
  /*
  const uint16_t AS5048A_CLEAR_ERROR_FLAG              = 0x0001;
  const uint16_t AS5048A_PROGRAMMING_CONTROL           = 0x0003;
  const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016;
  const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017;
  const uint16_t AS5048A_DIAG_AGC                      = 0x3FFD;
  const uint16_t AS5048A_MAGNITUDE                     = 0x3FFE;
  */
  const uint8_t AS5048A_ANGLE[2] = {0x3F, 0xFF};
  const uint8_t buf1[5] = "mag:";
  const uint8_t buf2[3] = "\r\n";
  static uint8_t spi_rx_buf[2];

  while(1) {
    chThdSleepMilliseconds(500);

    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
    spiExchange(&SPID1, 2, AS5048A_ANGLE, &spi_rx_buf);
    spiUnselect(&SPID1);

    spiReleaseBus(&SPID1);
    sdWrite(&SD1, &buf1[0], 4);
    sdWrite(&SD1, &spi_rx_buf[0], 2);
    sdWrite(&SD1, &buf2[0], 2);
  }
}

static THD_WORKING_AREA(waThread3, 128);
void Thread3(void) {
  chRegSetThreadName("motor");

  palSetPad(GPIOB, GPIOB_EN1);
  palSetPad(GPIOB, GPIOB_EN2);
  palSetPad(GPIOB, GPIOB_EN3);
  //palSetPad(GPIOC, GPIOC_RESET);
  //chThdSleepMilliseconds(50);
  //palClearPad(GPIOC, GPIOC_RESET);
  float half_pwr = 750;
    float angle = 0;
    while (TRUE) {
        if(angle < 6.28) {
            pwmEnableChannel(&PWMD3, 2, (half_pwr + half_pwr * sinf(angle)));
            pwmEnableChannel(&PWMD3, 1, (half_pwr + half_pwr * sinf(angle - 2.094)));
            pwmEnableChannel(&PWMD3, 0, (half_pwr + half_pwr * sinf(angle + 2.094)));
            angle += 0.1f;
        } else {
          angle = 0;
        }

        chThdSleepMilliseconds(1);
    }
}

static Node::uavcanNodeThread canNode;

int main(void) {
  halInit();
  chSysInit();
  sdStart(&SD1, &serialCfg);
  pwmStart(&PWMD3, &pwm_cfg);
  PWMD3.tim->CR1 |= STM32_TIM_CR1_CMS(1); //Set Center aligned mode

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, (tfunc_t)Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, (tfunc_t)Thread2, NULL);
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, (tfunc_t)Thread3, NULL);

  canNode.start(HIGHPRIO);

  while(1) {
    chThdSleepMilliseconds(500);
  }
}

