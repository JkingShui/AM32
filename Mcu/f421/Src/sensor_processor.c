#include "sensor_processor.h"
#include "lsm6ds3.h"
#include "at32f421_tmr.h"
#include "peripherals.h"
#include "targets.h"
#include "uart_print.h"

static sensor_data_t sensor_data;
extern uint16_t ADC_raw_volts;
extern uint16_t ADCDataDMA[4];
extern uint16_t ADC_raw_current;

volatile uint8_t pwm2_data_ready = 0;
uint32_t pwm2_capture_high_time = 0;

void sensor_processor_init(void) {
    sensor_data.gyro_z = 0;
    sensor_data.gyro_z_dps = 0.0f;
    sensor_data.pwm_high_time = 0;
    sensor_data.output_pwm = 0;
}

int32_t update_times = 0;
void sensor_processor_update_gyro(void) {
    if (++update_times >= 100) {
        update_times = 0;
        uint8_t ret = lsm6ds3_read_gyro_z(&sensor_data.gyro_z);
        if(ret != 1)
        {
            sensor_data.gyro_z_dps = 0.0f;
            return;
        }
        sensor_data.gyro_z_dps = lsm6ds3_convert_to_dps(sensor_data.gyro_z) * (180.0f / 3.1415926f);
    }
    
}

void sensor_processor_update_pwm_input(void) {
    if (pwm2_data_ready) {
        sensor_data.pwm_high_time = pwm2_capture_high_time;
        pwm2_data_ready = 0;
    }
}

int32_t times = 0;
uint32_t last_time;
void sensor_processor_calculate(void) {
    // float input_gyro = sensor_data.gyro_z_dps;
    uint32_t this_time = INTERVAL_TIMER_COUNT;
    
    // 每隔100ms输出一次 input_gyro    
    if (++times >= 1000) {  // 100ms = 100000us
        times = 0;
        // uart_print_number((int32_t)(this_time - last_time));
        uart_print_number((ADC_raw_volts * 3300 / 4095 * 11) / 100);
        uart_print_string("\n");
    }
    last_time = this_time;
    
    sensor_data.output_pwm = sensor_data.pwm_high_time;
}

void sensor_processor_update_output(void) {
    tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_1, sensor_data.output_pwm);
}

sensor_data_t* sensor_processor_get_data(void) {
    return &sensor_data;
}