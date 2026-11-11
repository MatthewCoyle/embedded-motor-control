#ifndef MOTOR_PLANT_H
#define MOTOR_PLANT_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_controller.h"

typedef struct {
    float shaft_rpm;
    float temperature_c;
    float current_a;
    float bus_v;
    float load;                 /* normalized 0..1 */
    bool jammed;
    bool sensor_online;
    bool force_temperature;
    bool force_current;
    bool force_speed;
    bool force_bus;
    float measured_temperature_c;
    float measured_current_a;
    float measured_speed_rpm;
    float measured_bus_v;
    uint32_t sensor_age_ms;
} motor_plant_t;

void plant_init(motor_plant_t *plant);
void plant_tick(motor_plant_t *plant, const motor_output_t *output, uint32_t dt_ms);
motor_sample_t plant_measure(const motor_plant_t *plant);
void plant_clear_injections(motor_plant_t *plant);

#endif
