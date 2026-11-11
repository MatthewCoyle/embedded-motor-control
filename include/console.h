#ifndef MOTOR_CONSOLE_H
#define MOTOR_CONSOLE_H

#include <stdbool.h>
#include <stdio.h>
#include "motor_controller.h"
#include "motor_plant.h"
#include "motor_hal.h"

typedef struct {
    motor_controller_t controller;
    motor_plant_t plant;
    motor_hal_t hal;
    motor_output_t applied_output;
} motor_app_t;

void app_init(motor_app_t *app);
void app_step(motor_app_t *app, uint32_t milliseconds);
void app_status(const motor_app_t *app, FILE *out);
/* Returns false for malformed or rejected command. */
bool app_execute(motor_app_t *app, const char *command, FILE *out);

#endif
