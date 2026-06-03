#ifndef __SENSOR_PROCESSOR_H
#define __SENSOR_PROCESSOR_H

#include <stdint.h>

typedef struct {
    int16_t gyro_z;
    float   gyro_z_dps;
    uint32_t pwm_high_time;
    uint16_t output_pwm;
} sensor_data_t;

void sensor_processor_init(void);
void sensor_processor_update_gyro(void);
void sensor_processor_update_pwm_input(void);
void sensor_processor_calculate(void);
void sensor_processor_update_output(void);
sensor_data_t* sensor_processor_get_data(void);

extern volatile uint8_t pwm2_data_ready;
extern uint32_t pwm2_capture_high_time;

#endif