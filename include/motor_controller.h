#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/* Software model only. These thresholds are demonstration settings, NOT rated
 * limits for any physical motor, driver, battery or safety system. */
#define MOTOR_TICK_MS               20U
#define MOTOR_MAX_TICK_GAP_MS       40U
#define MOTOR_STOPPED_RPM           50.0f
#define MOTOR_MAX_REQUEST_RPM       2800.0f
#define MOTOR_MIN_REQUEST_RPM       200.0f
#define MOTOR_MAX_PWM               1.0f
#define MOTOR_TEMP_LIMIT_C          80.0f
#define MOTOR_CURRENT_LIMIT_A       9.0f
#define MOTOR_BUS_MIN_V             18.0f
#define MOTOR_BUS_MAX_V             30.0f
#define MOTOR_STALL_DETECT_MS       800U
#define MOTOR_RECOVERY_MS          1000U
#define MOTOR_SENSOR_MAX_AGE_MS      40U
#define MOTOR_OVERSPEED_RPM        3300.0f
#define MOTOR_RESET_TEMP_C           70.0f
#define MOTOR_RESET_CURRENT_A         1.0f

typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_RUNNING,
    MOTOR_FAULT,
    MOTOR_RECOVERY
} motor_state_t;

typedef enum { DIR_FWD = 0, DIR_REV = 1 } motor_direction_t;

typedef enum {
    FAULT_NONE        = 0,
    FAULT_SENSOR      = 1u << 0,
    FAULT_OVER_TEMP   = 1u << 1,
    FAULT_OVER_CURRENT= 1u << 2,
    FAULT_BUS_VOLTAGE = 1u << 3,
    FAULT_STALL       = 1u << 4,
    FAULT_ESTOP       = 1u << 5,
    FAULT_TIMING      = 1u << 6,
    FAULT_OVERSPEED   = 1u << 7,
    FAULT_INTERNAL    = 1u << 8
} motor_fault_t;

typedef enum {
    CTRL_OK = 0,
    CTRL_BAD_VALUE,
    CTRL_BAD_STATE,
    CTRL_UNSAFE_SENSORS,
    CTRL_MOTOR_MOVING
} motor_result_t;

typedef struct {
    bool valid;
    float temperature_c;
    float current_a;
    float speed_rpm;  /* measured magnitude, irrespective of direction */
    float bus_v;
    uint32_t age_ms; /* acquisition age supplied by the HAL, not host wall time */
} motor_sample_t;

typedef struct {
    bool enable;
    motor_direction_t direction;
    float duty;      /* 0.0 to 1.0 */
} motor_output_t;

typedef struct {
    motor_state_t state;
    motor_direction_t direction;
    float requested_rpm;
    float ramped_rpm;
    float integral;
    uint32_t stall_elapsed_ms;
    uint32_t recovery_elapsed_ms;
    uint64_t uptime_ms;
    uint32_t faults;
    uint32_t fault_history; /* lifetime diagnostic OR mask, retained across RESET */
    bool have_sample;
    bool estop_asserted;
    motor_sample_t last_sample;
    motor_output_t output;
} motor_controller_t;

/* All controller pointers must be non-NULL and initialized with motor_init.
 * Single owner/task: concurrent calls require external synchronization.
 * motor_tick accepts a NULL sample as an acquisition failure. */
void motor_init(motor_controller_t *ctrl);
void motor_tick(motor_controller_t *ctrl, const motor_sample_t *sample, uint32_t dt_ms);
motor_result_t motor_start(motor_controller_t *ctrl);
motor_result_t motor_stop(motor_controller_t *ctrl);
motor_result_t motor_set_speed(motor_controller_t *ctrl, float rpm);
motor_result_t motor_set_direction(motor_controller_t *ctrl, motor_direction_t dir);
motor_result_t motor_reset(motor_controller_t *ctrl);
void motor_emergency_stop(motor_controller_t *ctrl);
void motor_release_estop(motor_controller_t *ctrl);
uint32_t motor_check_sample(const motor_sample_t *sample);
const char *motor_state_name(motor_state_t state);
const char *motor_result_name(motor_result_t result);

#endif
