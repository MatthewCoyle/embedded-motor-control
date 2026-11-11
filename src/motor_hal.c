#include "motor_hal.h"
#include <stddef.h>

bool motor_hal_cycle(motor_controller_t *controller, const motor_hal_t *hal,
                     uint32_t elapsed_ms)
{
    if (controller == NULL) return false;
    if (hal == NULL || hal->read == NULL || hal->write == NULL) {
        motor_emergency_stop(controller);
        /* Missing write callback cannot inhibit physical outputs. Board startup
           must validate callbacks and independently default the driver off. */
        if (hal != NULL && hal->write != NULL)
            hal->write(hal->context, &controller->output);
        return false;
    }
    motor_sample_t sample = {0};
    if (!hal->read(hal->context, &sample)) sample.valid = false;
    motor_tick(controller, &sample, elapsed_ms);
    hal->write(hal->context, &controller->output);
    return true;
}
