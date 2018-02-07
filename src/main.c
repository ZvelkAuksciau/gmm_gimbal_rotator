#include "ch.h"
#include "hal.h"

#include "canard.h"

//#include "stdlib.h"

//#include "km_math.h"
//#include "chprintf.h"

static SerialConfig serialCfg = {
  9600,
};

/*
 * Red LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);

static msg_t Thread1(void *arg) {
  (void)arg;

  uint8_t buf = '.';
  chRegSetThreadName("blinker");

  while (TRUE) {
    palClearPad(GPIOB, GPIOB_LED_RED);
    palClearPad(GPIOB, GPIOB_LED_GREEN);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOB, GPIOB_LED_RED);
    palSetPad(GPIOB, GPIOB_LED_GREEN);
    sdWrite(&SD1, &buf, 1);
    chThdSleepMilliseconds(500);
  }
}

int main(void) {
  halInit();
  chSysInit();
  sdStart(&SD1, &serialCfg);

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  chThdSleepMilliseconds(500);

  while(1){
    chThdSleepMilliseconds(500);
  }
}

