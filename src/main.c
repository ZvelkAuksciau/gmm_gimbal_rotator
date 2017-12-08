/*
 ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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

#include "ch.h"
#include "hal.h"
#include "stdlib.h"

#include "Storm32_telemetry.h"
#include "telemetry.h"
#include "parameters_d.h"
#include "km_math.h"
#include "chprintf.h"
#include "mavlink.h"

enum State_machine {
    WAITING_FOR_STORM_BOOTUP,
    CALIBRATING_ZERO_POSITION,
    WORKING,
}state;

static const SerialConfig sd3_config =
{
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

icucnt_t last_width, last_period;
icucnt_t pulse_width, pulse_period;

static void icuwidthcb(ICUDriver *icup) {
  last_width = icuGetWidthX(icup);
}

static void icuperiodcb(ICUDriver *icup) {
  last_period = icuGetPeriodX(icup);
  if(last_period > 21000 && last_period < 24000) {
    pulse_width = last_width;
    pulse_period = last_period;
  }
}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_LOW,
  20e6,                                    /* 10kHz ICU clock frequency.   */
  icuwidthcb,
  icuperiodcb,
  NULL,
  ICU_CHANNEL_1,
  0
};

#define MEDIAN_FILT_SIZE 9

float median_filter(float meas) { //only one thread can call (i think)
    static float meas_buff[MEDIAN_FILT_SIZE] = {0};
    static uint8_t data_pointer = 0; //Implement circular array

    meas_buff[data_pointer++] = meas;
    if(data_pointer == MEDIAN_FILT_SIZE) {
        data_pointer = 0;
    }
    float tmp_buff[MEDIAN_FILT_SIZE];
    memcpy(tmp_buff, meas_buff, sizeof(float) * MEDIAN_FILT_SIZE);

    //TODO: improve sorting algorithm
    //Sort buffer
    for(uint8_t i = 0; i < MEDIAN_FILT_SIZE-1; i++) {
        for(uint8_t j = i; j < MEDIAN_FILT_SIZE; j++) {
            if(tmp_buff[i] > tmp_buff[j]) {
                uint16_t tmp = tmp_buff[i];
                tmp_buff[i] = tmp_buff[j];
                tmp_buff[j] = tmp;
            }
        }
    }

    return tmp_buff[MEDIAN_FILT_SIZE/2];
}

int main(void) {
    halInit();
    chSysInit();

    chThdSetPriority(NORMALPRIO+2);
    /*
     * Activates the serial driver 1 using the driver default configuration.
     * PA9(TX) and PA10(RX) are routed to USART1.
     */
    sdStart(&SD1, NULL);
    sdStart(&SD3, &sd3_config);

    icuStart(&ICUD2, &icucfg);
    icuStartCapture(&ICUD2);
    icuEnableNotifications(&ICUD2);

    init_telemetry();

    init_sotmr32_telemetry();

    state = WAITING_FOR_STORM_BOOTUP;
    float target_yaw = 0.0f;
    float storm_last_reading = 0.0f;
    int32_t storm_rotation_count = 0;
    uint8_t count = 0;
    float last_encoder_pos = 0.0f;
    encoder_pos = 0.0f;

    while (TRUE) {
        //systime_t current_time = ST2MS(chVTGetSystemTime());
        //chprintf(&SD1, "%u %u\n", pulse_period, pulse_width);
        switch(state) {
            case WAITING_FOR_STORM_BOOTUP:
                if(count > 20) {
                  //palTogglePad(GPIOB, GPIOB_LED);
                  count = 0;
                }
                count++;
                if(get_storm_state()->state == 6) {
                    state = WORKING;
                    storm_rotation_count = 0;

                }
                break;
            case WORKING: {
                if(get_storm_state()->state != 6) {
                    state = WAITING_FOR_STORM_BOOTUP;
                    break;
                }
                float storm_yaw = get_storm_state()->imu1_yaw / 100.0f;
                float yaw_diff = storm_yaw - storm_last_reading;

                if(yaw_diff < -200.0f) {
                    storm_rotation_count++;
                } else if(yaw_diff > 200.0f) {
                    storm_rotation_count--;
                }

                storm_last_reading = storm_yaw;
                storm_yaw = wrap180(storm_yaw, 1) + 360.0f*storm_rotation_count;
                float enc_pos = wrap360((float)(pulse_width-42)/(float)(pulse_period-127) * 360.0f, 1);
                encoder_pos = wrap180(enc_pos, 1);


                if (mount_mode == MAV_MOUNT_MODE_MAVLINK_TARGETING && relative_yaw == 1) {
                    target_yaw = target_gimbal_yaw;
                } else {

                    float diff = wrap180(encoder_pos - target_gimbal_yaw, 1);
                    //float diff = encoder_pos - target_gimbal_yaw;
                    target_yaw = storm_yaw - diff;
                    //int target = target_yaw * 1000;
                }
                //chprintf(&SD1, "%i\n", target);


                set_target_angles(target_gimbal_pitch, 0.0f, target_yaw, true);
                //palTogglePad(GPIOB, GPIOB_LED);
                break;
            }
        }
        update_storm32();
        chThdSleepMilliseconds(20);
    }
}
