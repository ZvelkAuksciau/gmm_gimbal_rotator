#include <ch.h>
#include <hal.h>
#include <math.h>
#include "os.hpp"

#include <node.hpp>
#include <uavcan/equipment/hardpoint/Command.hpp>


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

#define HALF_POWER 750
#define FULL_POWER HALF_POWER*2

void pwmSetDCMotor(float cmd) {
    enableOutput();
    if(cmd > 0.0f && cmd <= 1.0f) {
        pwmEnableChannel(&PWMD3, 2, FULL_POWER*cmd);
        pwmEnableChannel(&PWMD3, 1, 0);
        pwmEnableChannel(&PWMD3, 0, 0);
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

    /*
    * Watchdog
    */
    os::watchdog::init();
    os::watchdog::Timer wdt;
    wdt.startMSec(5000);


    sdStart(&SD1, &serialCfg);
    pwmStart(&PWMD3, &pwm_cfg);
    PWMD3.tim->CR1 |= STM32_TIM_CR1_CMS(1); //Set Center aligned mode

    static os::stm32::ConfigStorageBackend config_storage_backend(ConfigStorageAddress, ConfigStorageSize);
    const int config_init_res = os::config::init(&config_storage_backend);

    nodeThd.start(NORMALPRIO-1);

    chThdSleepMilliseconds(2000);

    uavcan::Subscriber<uavcan::equipment::hardpoint::Command> comm_sub(Node::getNode());

    bool up = true;


    const int comm_sub_start_res =
            comm_sub.start(
                    [&](const uavcan::ReceivedDataStructure<uavcan::equipment::hardpoint::Command>& msg)
                    {
                        if(msg.hardpoint_id == 220 && msg.command == 1){
                            up = true;
                            Node::controling_node_id = msg.getSrcNodeID().get();
                        }
                        else if(msg.hardpoint_id == 220 && msg.command == 0){
                            up = false;
                            Node::controling_node_id = msg.getSrcNodeID().get();
                        }
                    });

    if(comm_sub_start_res < 0) {
      chSysHalt("Failed to start subscriber");
    }

    float down_power = 0.3f;
    float up_power = -0.6;
    float current_power = 0.0f;
    float startup_acc = 1.0f; //1.0f means full speed is reached in 1s
    float stop_acc = 0.2f;

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, (tfunc_t)Thread1, NULL);

  while(1) {
      wdt.reset();

      if(!Node::isControlingNodeAlive() && Node::controling_node_id != 0) {
          up = true;
      }

      if(!up) { //If gimbal is commanded to be down
          if(palReadPad(GPIOA, GPIOA_SPI1NSS)) //If end button is not pressed
          {
              if(fabs(current_power) < fabs(down_power)) {
                  current_power += down_power/(1000 * startup_acc);
              } else current_power = down_power;
          } else { //If end button is pressed
              if(fabs(current_power) > fabs(down_power/(1000 * stop_acc)*2.0f)) {
                  current_power -= down_power/(1000 * stop_acc);
              } else current_power = 0.0f;
          }
      } else { //If gimbal commanded to be up
          if(palReadPad(GPIOA, GPIOA_SPI1_MISO)) { //If end button is not pressed
              if(fabs(current_power) < fabs(up_power)) {
                  current_power += up_power/(1000.0f * startup_acc);
              } else current_power = up_power;
          } else { //If end button is pressed
              if(fabs(current_power) > fabs(up_power/(1000 * stop_acc)*2.0f)) {
                  current_power -= up_power/(1000 * stop_acc);
              } else current_power = 0.0f;
          }
      }
      pwmSetDCMotor(current_power);
      chThdSleepMilliseconds(1);
  }
}

