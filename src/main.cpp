#include <ch.h>
#include <hal.h>
#include <math.h>

#include <node.hpp>
#include <kmti/gimbal/GimbalCommand.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>

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

Node::uavcanNodeThread nodeThd;

static void* const ConfigStorageAddress = reinterpret_cast<void*>(0x08000000 + (128 * 1024) - 1024);
constexpr unsigned ConfigStorageSize = 1024;

void pwmSetDCMotor(float cmd) {
    enableOutput();
    if(cmd > 0.0f && cmd <= 1.0f) {
        pwmEnableChannel(&PWMD3, 0, FULL_POWER*cmd);
        pwmEnableChannel(&PWMD3, 1, 0);
        pwmEnableChannel(&PWMD3, 2, 0);
    } else if(cmd < 0.0f && cmd >= -1.0f) {
        pwmEnableChannel(&PWMD3, 0, 0);
        pwmEnableChannel(&PWMD3, 1, -FULL_POWER*cmd);
        pwmEnableChannel(&PWMD3, 2, 0);
    } else if(cmd == 0.0f) {
        pwmEnableChannel(&PWMD3, 0, 0);
        pwmEnableChannel(&PWMD3, 1, 0);
        pwmEnableChannel(&PWMD3, 2, 0);
    }


}

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

  chThdSleepMilliseconds(200);

  uavcan::Subscriber<kmti::gimbal::GimbalCommand> comm_sub(Node::getNode());

  bool up = true;


  const int comm_sub_start_res = comm_sub.start(
          [&](const uavcan::ReceivedDataStructure<kmti::gimbal::GimbalCommand>& msg)
          {
              if(msg.command == msg.COMMAND_RETRACT) up = true;
              else if(msg.command == msg.COMMAND_NORMAL) up = false;
          });

  if(comm_sub_start_res < 0) {
      chSysHalt("Failed to start subscriber");
  }

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, (tfunc_t)Thread1, NULL);

  while(1) {
      if(!up) {
          if(!palReadPad(GPIOA, GPIOA_SPI1NSS))
          {
              pwmSetDCMotor(1.0f);
          } else {
              pwmSetDCMotor(0.0f);
          }
      } else {
          if(!palReadPad(GPIOA, GPIOA_SPI1_MISO)) {
              pwmSetDCMotor(-1.0f);
          } else {
              pwmSetDCMotor(0.0f);
          }
      }
      chThdSleepMilliseconds(1);
  }
}

