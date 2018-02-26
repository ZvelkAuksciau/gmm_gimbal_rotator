#include <ch.h>
#include <hal.h>
#include <math.h>

#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <kmti/gimbal/MotorCOmmand.hpp>

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

    volatile systime_t time = chVTGetSystemTime();

    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
    spiExchange(&SPID1, 2, AS5048A_ANGLE, &spi_rx_buf);
    spiUnselect(&SPID1);

    spiReleaseBus(&SPID1);

    time = chVTGetSystemTime() - time;
    sdWrite(&SD1, &buf1[0], 4);
    sdWrite(&SD1, &spi_rx_buf[0], 2);
    sdWrite(&SD1, &buf2[0], 2);


  }
}

//static THD_WORKING_AREA(waThread3, 128);
//void Thread3(void) {
//  chRegSetThreadName("motor");
//
//  float half_pwr = 750;
//    float angle = 0;
//    while (TRUE) {
//        if(angle < 6.28) {
//            pwmEnableChannel(&PWMD3, 2, (half_pwr + half_pwr * sinf(angle)));
//            pwmEnableChannel(&PWMD3, 1, (half_pwr + half_pwr * sinf(angle - 2.094)));
//            pwmEnableChannel(&PWMD3, 0, (half_pwr + half_pwr * sinf(angle + 2.094)));
//            angle += 0.1f;
//        } else {
//          angle = 0;
//        }
//
//        chThdSleepMilliseconds(1);
//    }
//}

uavcan_stm32::CanInitHelper<> can;
constexpr unsigned NodePoolSize = 2000;

uavcan::Node<NodePoolSize>& getNode() {
  static uavcan::Node<NodePoolSize> node(can.driver, uavcan_stm32::SystemClock::instance());
  return node;
}

//Pitch 0;Roll 1 ;Yaw 2
#define AXIS 0
#define HALF_POWER 750

void disableOutput() {
    palClearPad(GPIOB, GPIOB_EN1);
    palClearPad(GPIOB, GPIOB_EN2);
    palClearPad(GPIOB, GPIOB_EN3);
    pwmDisableChannel(&PWMD3, 0);
    pwmDisableChannel(&PWMD3, 1);
    pwmDisableChannel(&PWMD3, 2);
}

systime_t lastCommandTime = 0;

int main(void) {
  halInit();
  chSysInit();

  palClearPad(GPIOB, GPIOB_EN1);
  palClearPad(GPIOB, GPIOB_EN2);
  palClearPad(GPIOB, GPIOB_EN3);

  sdStart(&SD1, &serialCfg);
  pwmStart(&PWMD3, &pwm_cfg);
  PWMD3.tim->CR1 |= STM32_TIM_CR1_CMS(1); //Set Center aligned mode


  uavcan::uint32_t bitrate = 1000000;
  can.init(bitrate);

  getNode().setName("org.kmti.gmm_controler");
  getNode().setNodeID(12);

  if (getNode().start() < 0) {
    chSysHalt("UAVCAN init fail");
  }

  uavcan::Subscriber<kmti::gimbal::MotorCommand> mot_sub(getNode());

  const int mot_sub_start_res = mot_sub.start(
          [&](const uavcan::ReceivedDataStructure<kmti::gimbal::MotorCommand>& msg)
          {
              if(msg.power[AXIS] <= 0) {
                  disableOutput();
              } else {
                  float cmd = msg.cmd[AXIS];
                  float power = HALF_POWER * msg.power[AXIS];
                  palSetPad(GPIOB, GPIOB_EN1);
                  palSetPad(GPIOB, GPIOB_EN2);
                  palSetPad(GPIOB, GPIOB_EN3);
                  pwmEnableChannel(&PWMD3, 2, (HALF_POWER + power * sinf(cmd)));
                  pwmEnableChannel(&PWMD3, 1, (HALF_POWER + power * sinf(cmd - 2.094)));
                  pwmEnableChannel(&PWMD3, 0, (HALF_POWER + power * sinf(cmd + 2.094)));
              }
              lastCommandTime = chVTGetSystemTime();
          });

  if(mot_sub_start_res < 0) {
      chSysHalt("Failed to start subscriber");
  }

  getNode().setModeOperational();

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, (tfunc_t)Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, (tfunc_t)Thread2, NULL);

  while(1) {
      if(getNode().spin(uavcan::MonotonicDuration::fromMSec(100)) < 0){
      }
      if(lastCommandTime != 0 && lastCommandTime + MS2ST(500) < chVTGetSystemTime()) {
          lastCommandTime = 0;
          disableOutput();
      }
  }
}

