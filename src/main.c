#include "ch.h"
#include "hal.h"

#include "canard.h"

//#include "stdlib.h"

//#include "km_math.h"
//#include "chprintf.h"

static SerialConfig serialCfg = {
  9600
};

/*
 * Low speed SPI configuration (281.250kHz, CPHA=1, CPOL=0, MSb first).
 */
static const SPIConfig ls_spicfg = {
  NULL,
  GPIOA,
  GPIOA_SPI1NSS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPHA
};

/*
 * SPI TX and RX buffers.
 */
static uint16_t spi_rx_buf;

const uint16_t AS5048A_CLEAR_ERROR_FLAG              = 0x0001;
const uint16_t AS5048A_PROGRAMMING_CONTROL           = 0x0003;
const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016;
const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017;
const uint16_t AS5048A_DIAG_AGC                      = 0x3FFD;
const uint16_t AS5048A_MAGNITUDE                     = 0x3FFE;
const uint16_t AS5048A_ANGLE                         = 0x3FFF;

/*
 * Red LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_WORKING_AREA(waThread2, 128);

static msg_t Thread1(void *arg) {
  (void)arg;

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

static msg_t Thread2(void *arg) {
  (void)arg;
  uint8_t buf = '.';
  int rotation;

  chRegSetThreadName("rotary_position_sensor");

  while(1) {
    chThdSleepMilliseconds(500);

    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
    spiExchange(&SPID1, 2, AS5048A_ANGLE, &spi_rx_buf);
    spiUnselect(&SPID1);
    spiReleaseBus(&SPID1);
    sdWrite(&SD1, &spi_rx_buf, 2);
    sdWrite(&SD1, &buf, 1);
  }
}

int main(void) {
  halInit();
  chSysInit();
  sdStart(&SD1, &serialCfg);

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  chThdSleepMilliseconds(500);

  while(1) {
    chThdSleepMilliseconds(500);
  }
}

