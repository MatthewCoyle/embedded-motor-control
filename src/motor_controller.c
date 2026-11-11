#include "motor_controller.h"
#include <math.h>
#include <stddef.h>

#define PI_KP                       0.65f
#define PI_KI                       0.35f
#define RAMP_RPM_PER_SECOND      1000.0f
#define PWM_CHANGE_PER_SECOND       2.0f
#define NOMINAL_FREE_RPM         3000.0f
#define STALL_MIN_DUTY              0.08f
#define STALL_SPEED_FRACTION        0.15f

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static void outputs_off(motor_controller_t *ctrl)
{
    ctrl->output.enable = false;
    ctrl->output.duty = 0.0f;
    ctrl->output.direction = ctrl->direction;
}

static void enter_fault(motor_controller_t *ctrl, uint32_t reason)
{
    ctrl->faults |= reason;
    ctrl->fault_history |= reason;
    ctrl->state = MOTOR_FAULT;
    ctrl->ramped_rpm = 0.0f;
    ctrl->integral = 0.0f;
    ctrl->stall_elapsed_ms = 0u;
    ctrl->recovery_elapsed_ms = 0u;
    outputs_off(ctrl); /* immediate inhibit within this control update */
}

uint32_t motor_check_sample(const motor_sample_t *s)
{
    if (s == NULL || !s->valid || s->age_ms > MOTOR_SENSOR_MAX_AGE_MS ||
        !isfinite(s->temperature_c) || !isfinite(s->current_a) ||
        !isfinite(s->speed_rpm) || !isfinite(s->bus_v) ||
        s->temperature_c < -40.0f || s->temperature_c > 150.0f ||
        s->current_a < 0.0f || s->current_a > 100.0f ||
        s->speed_rpm < 0.0f || s->speed_rpm > 10000.0f ||
        s->bus_v < 0.0f || s->bus_v > 60.0f) {
        return FAULT_SENSOR;
    }
    uint32_t result = FAULT_NONE;
    if (s->speed_rpm >= MOTOR_OVERSPEED_RPM) result |= FAULT_OVERSPEED;
    if (s->temperature_c >= MOTOR_TEMP_LIMIT_C) result |= FAULT_OVER_TEMP;
    if (s->current_a >= MOTOR_CURRENT_LIMIT_A) result |= FAULT_OVER_CURRENT;
    if (s->bus_v < MOTOR_BUS_MIN_V || s->bus_v > MOTOR_BUS_MAX_V)
        result |= FAULT_BUS_VOLTAGE;
    return result;
}

void motor_init(motor_controller_t *ctrl)
{
    *ctrl = (motor_controller_t){0};
    ctrl->state = MOTOR_IDLE;
    ctrl->direction = DIR_FWD;
    ctrl->requested_rpm = 1200.0f;
    outputs_off(ctrl);
}

motor_result_t motor_set_speed(motor_controller_t *ctrl, float rpm)
{
    if (!isfinite(rpm) || rpm < MOTOR_MIN_REQUEST_RPM ||
        rpm > MOTOR_MAX_REQUEST_RPM) return CTRL_BAD_VALUE;
    if (ctrl->state != MOTOR_IDLE && ctrl->state != MOTOR_RUNNING)
        return CTRL_BAD_STATE;
    ctrl->requested_rpm = rpm;
    return CTRL_OK;
}

motor_result_t motor_set_direction(motor_controller_t *ctrl, motor_direction_t dir)
{
    if (dir != DIR_FWD && dir != DIR_REV) return CTRL_BAD_VALUE;
    if (ctrl->state != MOTOR_IDLE) return CTRL_BAD_STATE;
    if (!ctrl->have_sample ||
        motor_check_sample(&ctrl->last_sample) != FAULT_NONE)
        return CTRL_UNSAFE_SENSORS;
    if (ctrl->last_sample.speed_rpm > MOTOR_STOPPED_RPM)
        return CTRL_MOTOR_MOVING;
    ctrl->direction = dir;
    ctrl->output.direction = dir;
    return CTRL_OK;
}

motor_result_t motor_start(motor_controller_t *ctrl)
{
    if (ctrl->state != MOTOR_IDLE) return CTRL_BAD_STATE;
    if (!ctrl->have_sample || motor_check_sample(&ctrl->last_sample) != FAULT_NONE)
        return CTRL_UNSAFE_SENSORS;
    if (ctrl->estop_asserted) return CTRL_BAD_STATE;
    if (ctrl->last_sample.speed_rpm > MOTOR_STOPPED_RPM)
        return CTRL_MOTOR_MOVING;
    ctrl->state = MOTOR_RUNNING;
    ctrl->ramped_rpm = 0.0f;
    ctrl->integral = 0.0f;
    ctrl->stall_elapsed_ms = 0u;
    return CTRL_OK;
}

motor_result_t motor_stop(motor_controller_t *ctrl)
{
    if (ctrl->state != MOTOR_IDLE && ctrl->state != MOTOR_RUNNING)
        return CTRL_BAD_STATE;
    ctrl->state = MOTOR_IDLE;
    ctrl->ramped_rpm = 0.0f;
    ctrl->integral = 0.0f;
    ctrl->stall_elapsed_ms = 0u;
    outputs_off(ctrl);
    return CTRL_OK;
}

void motor_emergency_stop(motor_controller_t *ctrl)
{
    ctrl->estop_asserted = true;
    enter_fault(ctrl, FAULT_ESTOP);
}

void motor_release_estop(motor_controller_t *ctrl)
{
    ctrl->estop_asserted = false; /* release never clears the latch or restarts */
}

static bool recovery_sample_safe(const motor_controller_t *ctrl)
{
    return ctrl->have_sample && !ctrl->estop_asserted &&
        motor_check_sample(&ctrl->last_sample) == FAULT_NONE &&
        ctrl->last_sample.temperature_c <= MOTOR_RESET_TEMP_C &&
        ctrl->last_sample.current_a <= MOTOR_RESET_CURRENT_A &&
        ctrl->last_sample.speed_rpm <= MOTOR_STOPPED_RPM;
}

motor_result_t motor_reset(motor_controller_t *ctrl)
{
    if (ctrl->state != MOTOR_FAULT) return CTRL_BAD_STATE;
    if (!recovery_sample_safe(ctrl))
        return CTRL_UNSAFE_SENSORS;
    ctrl->state = MOTOR_RECOVERY;
    ctrl->faults = FAULT_NONE;
    ctrl->recovery_elapsed_ms = 0u;
    outputs_off(ctrl);
    return CTRL_OK;
}

void motor_tick(motor_controller_t *ctrl, const motor_sample_t *sample, uint32_t dt_ms)
{
    if (ctrl->state < MOTOR_IDLE || ctrl->state > MOTOR_RECOVERY) {
        enter_fault(ctrl, FAULT_INTERNAL);
        return;
    }
    if (dt_ms == 0u || dt_ms > MOTOR_MAX_TICK_GAP_MS) {
        ctrl->have_sample = false; /* invalid scheduling must not leave startup armed */
        if (ctrl->state == MOTOR_RUNNING || ctrl->state == MOTOR_RECOVERY)
            enter_fault(ctrl, FAULT_TIMING);
        return;
    } /* missed timing budget: fail closed */
    ctrl->uptime_ms += dt_ms;
    if (sample != NULL) {
        ctrl->last_sample = *sample;
        ctrl->have_sample = true;
    } else {
        ctrl->have_sample = false;
    }

    if (ctrl->state == MOTOR_FAULT) {
        uint32_t additional = motor_check_sample(sample);
        ctrl->faults |= additional;
        ctrl->fault_history |= additional;
        outputs_off(ctrl);
        return;
    }
    if (ctrl->state == MOTOR_IDLE) {
        outputs_off(ctrl);
        return;
    }

    uint32_t observed_fault = motor_check_sample(sample);
    if (observed_fault != FAULT_NONE) {
        enter_fault(ctrl, observed_fault);
        return;
    }

    if (ctrl->state == MOTOR_RECOVERY) {
        outputs_off(ctrl);
        if (!recovery_sample_safe(ctrl)) {
            ctrl->recovery_elapsed_ms = 0u;
            return; /* require a new consecutive dwell, not a marginal reset */
        }
        ctrl->recovery_elapsed_ms += dt_ms;
        if (ctrl->recovery_elapsed_ms >= MOTOR_RECOVERY_MS) {
            ctrl->state = MOTOR_IDLE; /* never automatically restart */
            ctrl->recovery_elapsed_ms = 0u;
        }
        return;
    }

    /* RUNNING: feedforward + PI with basic conditional anti-windup and PWM slew. */
    float dt = (float)dt_ms / 1000.0f;
    float rpm_change = RAMP_RPM_PER_SECOND * dt;
    ctrl->ramped_rpm += clampf(ctrl->requested_rpm - ctrl->ramped_rpm,
                               -rpm_change, rpm_change);
    float error = (ctrl->ramped_rpm - sample->speed_rpm) / NOMINAL_FREE_RPM;
    float proposed_integral = clampf(ctrl->integral + PI_KI * error * dt,
                                     -0.40f, 0.40f);
    float unsaturated = ctrl->ramped_rpm / NOMINAL_FREE_RPM +
                        PI_KP * error + proposed_integral;
    float pwm_step = PWM_CHANGE_PER_SECOND * dt;
    float reachable_lo = fmaxf(0.0f, ctrl->output.duty - pwm_step);
    float reachable_hi = fminf(1.0f, ctrl->output.duty + pwm_step);
    /* Treat actuator slew as saturation too: integrate only when reachable,
       or when the error drives the demand back toward the reachable interval. */
    if ((unsaturated >= reachable_lo && unsaturated <= reachable_hi) ||
        (unsaturated > reachable_hi && error < 0.0f) ||
        (unsaturated < reachable_lo && error > 0.0f)) {
        ctrl->integral = proposed_integral;
    }
    float demand = clampf(ctrl->ramped_rpm / NOMINAL_FREE_RPM +
                          PI_KP * error + ctrl->integral, 0.0f, 1.0f);
    ctrl->output.duty += clampf(demand - ctrl->output.duty,
                                -pwm_step, pwm_step);
    ctrl->output.duty = clampf(ctrl->output.duty, 0.0f, MOTOR_MAX_PWM);
    ctrl->output.direction = ctrl->direction;
    ctrl->output.enable = true;

    float stall_effort = fmaxf(STALL_MIN_DUTY,
                               0.5f * ctrl->ramped_rpm / NOMINAL_FREE_RPM);
    if (ctrl->ramped_rpm >= MOTOR_MIN_REQUEST_RPM &&
        ctrl->output.duty >= stall_effort &&
        sample->speed_rpm < STALL_SPEED_FRACTION * ctrl->ramped_rpm) {
        ctrl->stall_elapsed_ms += dt_ms;
        if (ctrl->stall_elapsed_ms >= MOTOR_STALL_DETECT_MS)
            enter_fault(ctrl, FAULT_STALL);
    } else {
        ctrl->stall_elapsed_ms = 0u;
    }
}

const char *motor_state_name(motor_state_t state)
{
    switch (state) {
        case MOTOR_IDLE: return "IDLE";
        case MOTOR_RUNNING: return "RUNNING";
        case MOTOR_FAULT: return "FAULT";
        case MOTOR_RECOVERY: return "RECOVERY";
        default: return "UNKNOWN";
    }
}

const char *motor_result_name(motor_result_t result)
{
    switch (result) {
        case CTRL_OK: return "OK";
        case CTRL_BAD_VALUE: return "BAD_VALUE";
        case CTRL_BAD_STATE: return "BAD_STATE";
        case CTRL_UNSAFE_SENSORS: return "UNSAFE_SENSORS";
        case CTRL_MOTOR_MOVING: return "MOTOR_MOVING";
        default: return "UNKNOWN";
    }
}
