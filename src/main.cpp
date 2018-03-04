#include <ch.h>
#include <hal.h>
#include <math.h>

#include <node.hpp>
#include <kmti/gimbal/MotorCommand.hpp>
#include <kmti/gimbal/MotorStatus.hpp>

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
 * SPI configuration (9MHz, CPHA=1, CPOL=0, MSb first).
 */
static const SPIConfig spicfg = {
  NULL,
  GPIOA,
  GPIOA_SPI1NSS,
  SPI_CR1_BR_1 | SPI_CR1_CPHA,
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

#define BOARD_NORMAL                0x00
#define BOARD_CALIBRATING_NUM_POLES 0x01
#define BOARD_CALIBRATING_OFFSET    0x04

#define HALF_POWER 750

uint8_t g_boardStatus = 0;
float cmd_power = 0.0f;

void disableOutput() {
    palClearPad(GPIOB, GPIOB_EN1);
    palClearPad(GPIOB, GPIOB_EN2);
    palClearPad(GPIOB, GPIOB_EN3);
    pwmDisableChannel(&PWMD3, 0);
    pwmDisableChannel(&PWMD3, 1);
    pwmDisableChannel(&PWMD3, 2);
}

void enableOutput() {
  palSetPad(GPIOB, GPIOB_EN1);
  palSetPad(GPIOB, GPIOB_EN2);
  palSetPad(GPIOB, GPIOB_EN3);
}

void setPwmCommand(float cmd, float set_power) {
  float power = HALF_POWER * set_power;
  if(set_power >= 0.0f) {
    pwmEnableChannel(&PWMD3, 2, (HALF_POWER + power * sinf(cmd)));
    pwmEnableChannel(&PWMD3, 1, (HALF_POWER + power * sinf(cmd - 2.094)));
    pwmEnableChannel(&PWMD3, 0, (HALF_POWER + power * sinf(cmd + 2.094)));
  } else {
    pwmEnableChannel(&PWMD3, 1, (HALF_POWER - power * sinf(cmd)));
    pwmEnableChannel(&PWMD3, 2, (HALF_POWER - power * sinf(cmd - 2.094)));
    pwmEnableChannel(&PWMD3, 0, (HALF_POWER - power * sinf(cmd + 2.094)));

  }
}

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

//Running at 5khz
static THD_WORKING_AREA(waRotoryEncThd, 128);
void RotoryEncThd(void) {
  chRegSetThreadName("rotary_position_sensor");

  const uint8_t AS5048A_ANGLE[2] = {0xFF, 0xFF};
  uint8_t spi_rx_buf[2];
  uint16_t mot_pos;

  uint8_t g_numPoles = 0;
  int16_t offset = 0;
  int8_t reversed = 1;

  enum {
    GO_TO_ZERO,
    READ_ZERO_POS,
    DO_ONE_ROTATION,
  };
  uint8_t poleCalibState = GO_TO_ZERO;

  uint32_t zero_pos_avg = 0;
  uint8_t zero_pos_avg_count = 0;
  float cmd_angle = 0.0f;
  bool dir_calibrated = false;

  while(1) {
    systime_t time = chVTGetSystemTime() + US2ST(200);
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &spicfg);
    spiSelect(&SPID1);
    spiExchange(&SPID1, 2, AS5048A_ANGLE, &spi_rx_buf);
    spiUnselect(&SPID1);

    spiReleaseBus(&SPID1);
    mot_pos = spi_rx_buf[0] << 8 & 0x3F00;
    mot_pos |= spi_rx_buf[1] & 0x00FF;

    if(g_boardStatus == BOARD_NORMAL) {
      if(cmd_power > 0.0f) {
        enableOutput();
        float command = (int32_t)(mot_pos - zero_pos_avg) * 0.00038349519f * g_numPoles  + reversed * 1.570796f;
        setPwmCommand(command, cmd_power);
      } else if(cmd_power < 0.0f) {
        enableOutput();
        float command = (int32_t)(mot_pos - zero_pos_avg) * 0.00038349519f * g_numPoles  - reversed * 1.570796f;
        setPwmCommand(command, -cmd_power);
      } else {
        disableOutput();
      }
    }

    if(g_boardStatus & BOARD_CALIBRATING_NUM_POLES) {
      switch(poleCalibState){
      case GO_TO_ZERO:
        cmd_angle = 0.0f;
        enableOutput();
        setPwmCommand(0.0f, 0.3f);
        chThdSleepMilliseconds(1000);
        poleCalibState = READ_ZERO_POS;
        break;
      case READ_ZERO_POS:
        zero_pos_avg += mot_pos;
        zero_pos_avg_count++;
        if(zero_pos_avg_count == 100) {
          zero_pos_avg /= zero_pos_avg_count;
          poleCalibState = DO_ONE_ROTATION;
        }
        break;
      case DO_ONE_ROTATION:
        cmd_angle += 0.005f;
        setPwmCommand(cmd_angle, 0.4f);
        int32_t diff = mot_pos - zero_pos_avg;
        if(cmd_angle > 1.0f && !dir_calibrated) {
          if(diff > 0) {
            reversed = 1;
            dir_calibrated = true;
          } else if(diff < 0) {
            reversed = -1;
            dir_calibrated = true;
          }
        }
        if(cmd_angle > 3.0f) {
          if(diff < 100 && diff > -100) {
            g_boardStatus &= ~BOARD_CALIBRATING_NUM_POLES;
            poleCalibState = GO_TO_ZERO;
            g_numPoles = round(cmd_angle/6.2831f);
          }
        }
        break;
      }
    }
    chThdSleepMicroseconds(200);
  }
}

//Pitch 0;Roll 1 ;Yaw 2
#define AXIS 2

systime_t lastCommandTime = 0;
Node::uavcanNodeThread nodeThd;

int main(void) {
  halInit();
  chSysInit();

  disableOutput();

  sdStart(&SD1, &serialCfg);
  pwmStart(&PWMD3, &pwm_cfg);
  PWMD3.tim->CR1 |= STM32_TIM_CR1_CMS(1); //Set Center aligned mode
  nodeThd.start(NORMALPRIO-1);

  g_boardStatus |= BOARD_CALIBRATING_NUM_POLES; //| BOARD_CALIBRATING_OFFSET;
  chThdSleepMilliseconds(200);

  uavcan::Subscriber<kmti::gimbal::MotorCommand> mot_sub(Node::getNode());

  const int mot_sub_start_res = mot_sub.start(
          [&](const uavcan::ReceivedDataStructure<kmti::gimbal::MotorCommand>& msg)
          {
              cmd_power = msg.cmd[AXIS];
              lastCommandTime = chVTGetSystemTime();
          });

  if(mot_sub_start_res < 0) {
      chSysHalt("Failed to start subscriber");
  }

  uavcan::Publisher<kmti::gimbal::MotorStatus> status_pub(Node::getNode());
  status_pub.init();
  kmti::gimbal::MotorStatus status_msg;
  status_msg.axis_id = AXIS;

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, (tfunc_t)Thread1, NULL);
  chThdCreateStatic(waRotoryEncThd, sizeof(waRotoryEncThd), NORMALPRIO+1, (tfunc_t)RotoryEncThd, NULL);

  while(1) {
      if(lastCommandTime != 0 && lastCommandTime + MS2ST(200) < chVTGetSystemTime()) {
          lastCommandTime = 0;
          disableOutput();
          cmd_power = 0.0f;
      }
      chThdSleepMilliseconds(100);
      //status_msg.motor_pos = mot_pos * 0.00038349519f;
     //status_pub.broadcast(status_msg);
  }
}

