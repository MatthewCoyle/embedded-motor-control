#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H
#include "motor_controller.h"

/* Caller owns context and callbacks; no allocation. All callbacks run in one
 * control-task context. A real board adapter must bound acquisition/write time. */
typedef struct {
    void *context;
    bool (*read)(void *context, motor_sample_t *sample);
    void (*write)(void *context, const motor_output_t *output);
} motor_hal_t;

/* False: invalid HAL contract. Failed acquisition is passed as invalid sample
 * and the resulting output is still written, so fault handling inhibits drive. */
bool motor_hal_cycle(motor_controller_t *controller, const motor_hal_t *hal,
                     uint32_t elapsed_ms);
#endif
