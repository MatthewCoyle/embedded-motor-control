#include "motor_plant.h"
#include <math.h>
#include <stddef.h>

static float clampf(float value, float lo, float hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

void plant_init(motor_plant_t *p)
{
    *p = (motor_plant_t){0};
    p->temperature_c = 25.0f;
    p->bus_v = 24.0f;
    p->load = 0.20f;
    p->sensor_age_ms = 0u;
    p->sensor_online = true;
}

void plant_clear_injections(motor_plant_t *p)
{
    p->jammed = false;
    p->sensor_online = true;
    p->force_temperature = false;
    p->force_current = false;
    p->force_speed = false;
    p->force_bus = false;
    p->load = 0.20f;
    p->sensor_age_ms = 0u;
}

void plant_tick(motor_plant_t *p, const motor_output_t *out, uint32_t dt_ms)
{
    float dt = (float)dt_ms / 1000.0f;
    float duty = (out != NULL && out->enable) ? clampf(out->duty, 0.0f, 1.0f) : 0.0f;
    float target = duty * 3000.0f * (1.0f - 0.5f * p->load);
    if (p->jammed) {
        p->shaft_rpm = 0.0f;
    } else {
        float alpha = dt / (0.25f + dt);
        p->shaft_rpm += (target - p->shaft_rpm) * alpha;
    }
    if (duty > 0.0f) {
        p->current_a = p->jammed ? (6.0f + duty) :
            (0.5f + 1.2f * duty + 3.0f * p->load +
             fabsf(target - p->shaft_rpm) / 3000.0f);
    } else {
        p->current_a = 0.0f;
    }
    /* Deliberately simplified thermal approximation for demonstration. */
    float heat_rate = 0.04f * p->current_a * p->current_a;
    float cooling_rate = 0.06f * (p->temperature_c - 25.0f);
    p->temperature_c += (heat_rate - cooling_rate) * dt;
}

motor_sample_t plant_measure(const motor_plant_t *p)
{
    motor_sample_t sample = {0};
    sample.valid = p->sensor_online;
    sample.temperature_c = p->force_temperature ? p->measured_temperature_c : p->temperature_c;
    sample.current_a = p->force_current ? p->measured_current_a : p->current_a;
    sample.speed_rpm = p->force_speed ? p->measured_speed_rpm : p->shaft_rpm;
    sample.bus_v = p->force_bus ? p->measured_bus_v : p->bus_v;
    sample.age_ms = p->sensor_age_ms;
    return sample;
}
