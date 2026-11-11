#include "console.h"
#include <stdio.h>

/* Export a deterministic and clearly labeled SIMULATION trace to CSV.
 * Program changes setpoint at 4 s, jams the shaft at 9 s. */
int main(void)
{
    motor_app_t a;
    app_init(&a);
    motor_set_speed(&a.controller, 1600.0f);
    if (motor_start(&a.controller) != CTRL_OK) return 1;
    puts("time_ms,requested_rpm,measured_rpm,pwm,current_a,temperature_c,state,simulated_jam");
    for (unsigned elapsed = 0u; elapsed <= 12500u; elapsed += MOTOR_TICK_MS) {
        if (elapsed == 4000u) motor_set_speed(&a.controller, 2200.0f);
        if (elapsed == 9000u) a.plant.jammed = true;
        motor_sample_t s = plant_measure(&a.plant);
        printf("%u,%.1f,%.2f,%.4f,%.3f,%.3f,%s,%d\n",
               elapsed, a.controller.requested_rpm, s.speed_rpm,
               a.controller.output.duty, s.current_a, s.temperature_c,
               motor_state_name(a.controller.state), a.plant.jammed ? 1 : 0);
        if (elapsed < 12500u) app_step(&a, MOTOR_TICK_MS);
    }
    return a.controller.state == MOTOR_FAULT && (a.controller.faults & FAULT_STALL) ? 0 : 1;
}
