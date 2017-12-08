#ifndef SRC_PARAMETERS_D_H_
#define SRC_PARAMETERS_D_H_
#include "ch.h"
#include "hal.h"

#include "telemetry.h"

//////////////////////////////////////////////////////////////////
// STOP!!! DO NOT CHANGE THIS VALUE UNTIL YOU FULLY UNDERSTAND THE
// COMMENTS ABOVE. IF UNSURE, ASK ANOTHER DEVELOPER!!!
//static const uint16_t k_format_version = 1;
//////////////////////////////////////////////////////////////////


// EEPROM layout
enum {
    k_param_format_version = 0,
    k_param_stream_param,
    k_param_stream_sensor,
    k_param_stream_rc_chan,
    k_param_stream_controller,
    k_param_encoder_offset,
    k_param_stream_req_data,
};

#define stream_param 10
#define stream_sensor 5
#define stream rc_chan 5
#define stream_contoller 5
#define encoder_offset 0.0f
#define stream_req_data 0.0

//extern float encoder_offset;

//void load_parameters(void);


#endif /* SRC_PARAMETERS_D_H_ */
