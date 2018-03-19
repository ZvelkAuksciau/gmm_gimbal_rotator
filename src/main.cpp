#include <ch.h>
#include <hal.h>
#include <math.h>

#include <node.hpp>
#include <kmti/gimbal/MotorCommand.hpp>

#include <config/config_storage_flash.hpp>
#include <config/config.hpp>

#define M_PI 3.1415926f
#define M_2PI 2*M_PI


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

float wrap_2PI(float radian)
{
    float res = fmodf(radian, M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

float wrap_PI(float radian)
{
    float res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
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

os::config::Param<uint8_t> num_poles("mot.num_poles", 7, 1, 255);
os::config::Param<float> enc_offset("mot.offset", 0.0f, -M_PI, M_PI);
os::config::Param<int8_t> direction("mot.dir", 1, -1, 1);

//Running at 5khz
static THD_WORKING_AREA(waRotoryEncThd, 128);
void RotoryEncThd(void) {
  chRegSetThreadName("rotary_position_sensor");

  const uint8_t AS5048A_ANGLE[2] = {0xFF, 0xFF};
  uint8_t spi_rx_buf[2];
  uint16_t mot_pos = 0;
  float mot_pos_rad = 0.0f;

  enum {
    GO_TO_ZERO,
    READ_ZERO_POS,
    DO_ONE_ROTATION,
    DO_4_ROTATIONS,
    DO_4_REV_ROTATIONS,
  };
  uint8_t calibState = GO_TO_ZERO;

  int32_t avg_calc = 0;
  uint32_t avg_count = 0;
  float off_avg = 0.0f;
  float cmd_angle = 0.0f;
  bool dir_calibrated = false;

  volatile uint8_t num_pol = num_poles.get();
  volatile float en_off = enc_offset.get();
  volatile int8_t rev = direction.get();

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
    mot_pos_rad = mot_pos * 0.00038349519f;

    if(g_boardStatus == BOARD_NORMAL) {
      if(cmd_power > 0.0f) {
        enableOutput();
        float command = mot_pos_rad * num_poles.get() - enc_offset.get() + direction.get() * 1.570796f;
        setPwmCommand(command, cmd_power);
      } else if(cmd_power < 0.0f) {
        enableOutput();
        float command = mot_pos_rad * num_poles.get() - enc_offset.get()  - direction.get() * 1.570796f;
        setPwmCommand(command, -cmd_power);
      } else {
        disableOutput();
      }
    }else if(g_boardStatus & BOARD_CALIBRATING_NUM_POLES) {
      switch(calibState){
      case GO_TO_ZERO:
        cmd_angle = 0.0f;
        avg_count = 0;
        avg_calc = 0;
        enableOutput();
        setPwmCommand(0.0f, 0.3f);
        chThdSleepMilliseconds(1000);
        calibState = READ_ZERO_POS;
        break;
      case READ_ZERO_POS:
        avg_calc += mot_pos;
        avg_count++;
        if(avg_count == 100) {
          avg_calc /= avg_count;
          calibState = DO_ONE_ROTATION;
        }
        break;
      case DO_ONE_ROTATION:
        cmd_angle += 0.005f;
        setPwmCommand(cmd_angle, 0.4f);
        int32_t diff = mot_pos - avg_calc;
        if(cmd_angle > 1.0f && !dir_calibrated) {
          if(diff > 0) {
            Node::publishKeyValue("mot_dir", 1.0f);
            direction.set(1);
            dir_calibrated = true;
          } else if(diff < 0) {
            Node::publishKeyValue("mot_dir", -1.0f);
            direction.set(-1);
            dir_calibrated = true;
          }
        }
        if(cmd_angle > 3.0f) {
          if(diff < 100 && diff > -100) {
            g_boardStatus &= ~BOARD_CALIBRATING_NUM_POLES;
            calibState = GO_TO_ZERO;
            disableOutput();
            num_poles.set(round(cmd_angle/6.2831f));
            Node::publishKeyValue("mot_poles", num_poles.get());
            os::config::save();
          }
        }
        break;
      }
    } else if(g_boardStatus & BOARD_CALIBRATING_OFFSET) {
      switch(calibState){
      case GO_TO_ZERO:
        enableOutput();
        avg_count = 0;
        off_avg = 0.0f;
        setPwmCommand(0.0f, 0.4f);
        chThdSleepMilliseconds(1000);
        cmd_angle = 0.0f;
        calibState = DO_4_ROTATIONS;
        break;
      case DO_4_ROTATIONS:
        off_avg += wrap_2PI(mot_pos_rad*num_poles.get() - cmd_angle);
        avg_count++;
        cmd_angle += direction.get() * 0.005f;
        setPwmCommand(wrap_2PI(cmd_angle), 0.4f);
        if(cmd_angle >= 4*M_2PI*num_poles.get()) {
          off_avg /= avg_count;
          Node::publishKeyValue("mot_pos_off", off_avg);
          enc_offset.set(off_avg);
          off_avg = 0.0f;
          avg_count = 0;
          calibState = DO_4_REV_ROTATIONS;
          cmd_angle = 0.0f;
        }
        break;
      case DO_4_REV_ROTATIONS:
        off_avg += wrap_2PI(mot_pos_rad*num_poles.get() - cmd_angle);
        avg_count++;
        cmd_angle -= direction.get() * 0.005f;
        setPwmCommand(wrap_2PI(cmd_angle), 0.4f);
        if(cmd_angle <= -4*M_2PI*num_poles.get()) {
          off_avg /= avg_count;
          Node::publishKeyValue("mot_neg_off", off_avg);
          enc_offset.set(enc_offset.get()/2.0f + off_avg/2.0f);
          off_avg = 0.0f;
          avg_count = 0;
          calibState = GO_TO_ZERO;
          g_boardStatus &= ~BOARD_CALIBRATING_OFFSET;
          os::config::save();
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

static void* const ConfigStorageAddress = reinterpret_cast<void*>(0x08000000 + (128 * 1024) - 1024);
constexpr unsigned ConfigStorageSize = 1024;

int main(void) {
  halInit();
  chSysInit();

  disableOutput();

  sdStart(&SD1, &serialCfg);
  pwmStart(&PWMD3, &pwm_cfg);
  PWMD3.tim->CR1 |= STM32_TIM_CR1_CMS(1); //Set Center aligned mode

  static os::stm32::ConfigStorageBackend config_storage_backend(ConfigStorageAddress, ConfigStorageSize);
  const int config_init_res = os::config::init(&config_storage_backend);

  nodeThd.start(NORMALPRIO-1);

  g_boardStatus |= BOARD_CALIBRATING_OFFSET | BOARD_CALIBRATING_NUM_POLES; //| BOARD_CALIBRATING_OFFSET;
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

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, (tfunc_t)Thread1, NULL);
  chThdCreateStatic(waRotoryEncThd, sizeof(waRotoryEncThd), NORMALPRIO+10, (tfunc_t)RotoryEncThd, NULL);

  while(1) {
      if(lastCommandTime != 0 && lastCommandTime + MS2ST(200) < chVTGetSystemTime()) {
          lastCommandTime = 0;
          disableOutput();
          cmd_power = 0.0f;
      }
      chThdSleepMilliseconds(100);
  }
}

