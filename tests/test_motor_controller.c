#include "console.h"
#include "motor_controller.h"
#include "motor_plant.h"
#include "uart_protocol.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "  CHECK FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    return false; } } while (0)

static motor_sample_t good_sample(void)
{
    motor_sample_t sample = {true, 25.0f, 0.2f, 0.0f, 24.0f, 0u};
    return sample;
}

static void ready(motor_controller_t *c)
{
    motor_init(c);
    motor_sample_t s = good_sample();
    motor_tick(c, &s, MOTOR_TICK_MS);
}

static bool test_initial_state(void)
{
    motor_controller_t c;
    motor_init(&c);
    CHECK(c.state == MOTOR_IDLE);
    CHECK(c.output.enable == false && c.output.duty == 0.0f);
    CHECK(c.requested_rpm == 1200.0f);
    CHECK(motor_start(&c) == CTRL_UNSAFE_SENSORS);
    return true;
}

static bool test_start_stop(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_start(&c) == CTRL_OK);
    motor_sample_t s = good_sample();
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_RUNNING && c.output.enable);
    CHECK(c.output.duty > 0.0f);
    CHECK(motor_stop(&c) == CTRL_OK);
    CHECK(c.state == MOTOR_IDLE && !c.output.enable && c.output.duty == 0.0f);
    return true;
}

static bool test_start_rejects_unsafe_reading(void)
{
    motor_controller_t c;
    ready(&c);
    motor_sample_t s = good_sample();
    s.temperature_c = MOTOR_TEMP_LIMIT_C;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(motor_start(&c) == CTRL_UNSAFE_SENSORS);
    CHECK(!c.output.enable);
    return true;
}

static bool test_speed_limits(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_set_speed(&c, 0.0f) == CTRL_BAD_VALUE);
    CHECK(motor_set_speed(&c, 2810.0f) == CTRL_BAD_VALUE);
    CHECK(motor_set_speed(&c, NAN) == CTRL_BAD_VALUE);
    CHECK(motor_set_speed(&c, 200.0f) == CTRL_OK);
    CHECK(motor_set_speed(&c, 2200.0f) == CTRL_OK);
    CHECK(c.requested_rpm == 2200.0f);
    return true;
}

static bool test_direction_locked_while_running(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_set_direction(&c, DIR_REV) == CTRL_OK);
    CHECK(motor_start(&c) == CTRL_OK);
    CHECK(motor_set_direction(&c, DIR_FWD) == CTRL_BAD_STATE);
    CHECK(c.direction == DIR_REV);
    CHECK(motor_stop(&c) == CTRL_OK);
    CHECK(motor_set_direction(&c, DIR_FWD) == CTRL_OK);
    return true;
}

static bool test_pwm_slew_bound(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_set_speed(&c, 2800.0f) == CTRL_OK);
    CHECK(motor_start(&c) == CTRL_OK);
    motor_sample_t s = good_sample();
    float last = 0.0f;
    for (int i = 0; i < 30; ++i) {
        motor_tick(&c, &s, MOTOR_TICK_MS);
        CHECK(c.state == MOTOR_RUNNING);
        CHECK(c.output.duty >= 0.0f && c.output.duty <= 1.0f);
        CHECK(fabsf(c.output.duty - last) <= 0.0401f);
        last = c.output.duty;
    }
    return true;
}

static bool test_fault_overcurrent_latches(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_start(&c) == CTRL_OK);
    motor_sample_t s = good_sample();
    s.current_a = 12.0f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_FAULT && (c.faults & FAULT_OVER_CURRENT));
    CHECK(!c.output.enable && c.output.duty == 0.0f);
    s.current_a = 0.0f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_FAULT); /* fault cannot self-clear */
    CHECK(motor_start(&c) == CTRL_BAD_STATE);
    return true;
}

static bool test_fault_overtemp(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_start(&c) == CTRL_OK);
    motor_sample_t s = good_sample();
    s.temperature_c = 81.0f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_FAULT && (c.faults & FAULT_OVER_TEMP));
    CHECK(!c.output.enable);
    return true;
}

static bool test_fault_bus(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_start(&c) == CTRL_OK);
    motor_sample_t s = good_sample();
    s.bus_v = 17.9f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_FAULT && (c.faults & FAULT_BUS_VOLTAGE));
    return true;
}

static bool test_fault_invalid_sensor(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_start(&c) == CTRL_OK);
    motor_sample_t s = good_sample();
    s.valid = false;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_FAULT && (c.faults & FAULT_SENSOR));
    return true;
}

static bool test_fault_nan_sensor(void)
{
    motor_sample_t s = good_sample();
    s.speed_rpm = NAN;
    CHECK(motor_check_sample(&s) == FAULT_SENSOR);
    s = good_sample();
    s.current_a = -1.0f;
    CHECK(motor_check_sample(&s) == FAULT_SENSOR);
    CHECK(motor_check_sample(NULL) == FAULT_SENSOR);
    return true;
}

static bool test_simulated_motor_normal(void)
{
    motor_app_t a;
    app_init(&a);
    CHECK(motor_set_speed(&a.controller, 1600.0f) == CTRL_OK);
    CHECK(motor_start(&a.controller) == CTRL_OK);
    app_step(&a, 10000u);
    CHECK(a.controller.state == MOTOR_RUNNING);
    CHECK(a.controller.faults == FAULT_NONE);
    motor_sample_t s = plant_measure(&a.plant);
    CHECK(s.speed_rpm > 1300.0f && s.speed_rpm < 1950.0f);
    CHECK(a.controller.output.duty <= 1.0f);
    return true;
}

static bool test_stall_on_mechanical_jam(void)
{
    motor_app_t a;
    app_init(&a);
    CHECK(motor_set_speed(&a.controller, 1800.0f) == CTRL_OK);
    CHECK(motor_start(&a.controller) == CTRL_OK);
    app_step(&a, 3000u);
    CHECK(a.controller.state == MOTOR_RUNNING);
    a.plant.jammed = true;
    app_step(&a, 3500u);
    CHECK(a.controller.state == MOTOR_FAULT);
    CHECK((a.controller.faults & FAULT_STALL) != 0u);
    CHECK(a.controller.output.duty == 0.0f);
    return true;
}

static bool test_reset_blocked_while_hot(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_start(&c) == CTRL_OK);
    motor_sample_t s = good_sample();
    s.temperature_c = 90.0f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(motor_reset(&c) == CTRL_UNSAFE_SENSORS);
    CHECK(c.state == MOTOR_FAULT);
    return true;
}

static bool test_reset_requires_cooldown_and_manual_start(void)
{
    motor_controller_t c;
    ready(&c);
    motor_emergency_stop(&c);
    CHECK(c.state == MOTOR_FAULT && (c.faults & FAULT_ESTOP));
    CHECK(!c.output.enable);
    motor_release_estop(&c); CHECK(motor_reset(&c) == CTRL_OK);
    CHECK(c.state == MOTOR_RECOVERY);
    CHECK(motor_start(&c) == CTRL_BAD_STATE);
    motor_sample_t s = good_sample();
    for (int i = 0; i < 49; ++i) motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_RECOVERY && !c.output.enable);
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_IDLE && !c.output.enable);
    CHECK(motor_start(&c) == CTRL_OK);
    return true;
}

static bool test_recovery_invalid_faults_again(void)
{
    motor_controller_t c;
    ready(&c);
    motor_emergency_stop(&c);
    motor_release_estop(&c); CHECK(motor_reset(&c) == CTRL_OK);
    motor_sample_t s = good_sample();
    s.bus_v = 34.0f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_FAULT);
    CHECK((c.faults & FAULT_BUS_VOLTAGE) != 0u);
    return true;
}

static bool test_estop_synchronous(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_start(&c) == CTRL_OK);
    motor_sample_t s = good_sample();
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.output.enable);
    motor_emergency_stop(&c);
    CHECK(!c.output.enable && c.output.duty == 0.0f);
    CHECK(c.state == MOTOR_FAULT);
    return true;
}

static bool test_missed_tick_fault(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_start(&c) == CTRL_OK);
    motor_sample_t s = good_sample();
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.output.enable);
    motor_tick(&c, &s, 100u);
    CHECK(c.state == MOTOR_FAULT);
    CHECK((c.faults & FAULT_TIMING) != 0u && !c.output.enable);
    return true;
}

static bool test_reverse_requires_stopped_rotor(void)
{
    motor_controller_t c;
    ready(&c);
    motor_sample_t s = good_sample();
    s.speed_rpm = 700.0f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(motor_set_direction(&c, DIR_REV) == CTRL_MOTOR_MOVING);
    s.speed_rpm = 40.0f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(motor_set_direction(&c, DIR_REV) == CTRL_OK);
    return true;
}

static bool test_parser_normal_crlf(void)
{
    uart_parser_t p;
    uart_init(&p);
    const char *input = "SPEED 1500\r\nSTART\n";
    int count = 0;
    for (size_t i = 0; input[i]; ++i) {
        uart_event_t e = uart_feed(&p, input[i]);
        if (e == UART_LINE_READY) {
            if (count == 0) CHECK(strcmp(p.line, "SPEED 1500") == 0);
            if (count == 1) CHECK(strcmp(p.line, "START") == 0);
            ++count;
        }
    }
    CHECK(count == 2);
    return true;
}

static bool test_parser_overflow_recovers(void)
{
    uart_parser_t p;
    uart_init(&p);
    for (int i = 0; i < 300; ++i) CHECK(uart_feed(&p, 'X') == UART_PENDING);
    CHECK(uart_feed(&p, '\n') == UART_LINE_OVERFLOW);
    const char *command = "STATUS\n";
    uart_event_t last = UART_PENDING;
    for (size_t i = 0; command[i]; ++i) last = uart_feed(&p, command[i]);
    CHECK(last == UART_LINE_READY && strcmp(p.line, "STATUS") == 0);
    return true;
}

static bool test_injections_clear(void)
{
    motor_plant_t p;
    plant_init(&p);
    p.jammed = true;
    p.sensor_online = false;
    p.force_current = true;
    p.measured_current_a = 99.0f;
    plant_clear_injections(&p);
    motor_sample_t s = plant_measure(&p);
    CHECK(!p.jammed && s.valid && !p.force_current);
    CHECK(s.current_a == 0.0f);
    return true;
}

static bool test_command_interface(void)
{
    motor_app_t a;
    app_init(&a);
    FILE *sink = tmpfile();
    CHECK(sink != NULL);
    CHECK(app_execute(&a, "SPEED 1500", sink));
    CHECK(app_execute(&a, "START", sink));
    CHECK(!app_execute(&a, "DIR REV", sink));
    CHECK(!app_execute(&a, "SPEED 5000", sink));
    CHECK(!app_execute(&a, "STEP 13", sink));
    CHECK(!app_execute(&a, "START EXTRA", sink));
    CHECK(app_execute(&a, "INJECT TEMP 91", sink));
    CHECK(app_execute(&a, "STEP 20", sink));
    CHECK(a.controller.state == MOTOR_FAULT);
    CHECK(app_execute(&a, "INJECT CLEAR", sink));
    CHECK(app_execute(&a, "STEP 20", sink));
    CHECK(app_execute(&a, "RESET", sink));
    CHECK(app_execute(&a, "STEP 1000", sink));
    CHECK(a.controller.state == MOTOR_IDLE);
    CHECK(app_execute(&a, "START", sink));
    CHECK(app_execute(&a, "STEP 1000", sink));
    CHECK(a.applied_output.enable);
    CHECK(app_execute(&a, "STOP", sink));
    CHECK(!a.applied_output.enable && a.applied_output.duty == 0.0f);
    CHECK(app_execute(&a, "START", sink) == false); /* still coasting */
    CHECK(app_execute(&a, "STEP 1200", sink));
    CHECK(app_execute(&a, "START", sink));
    CHECK(app_execute(&a, "STEP 1000", sink));
    CHECK(app_execute(&a, "ESTOP", sink));
    CHECK(!a.applied_output.enable && a.applied_output.duty == 0.0f);
    fclose(sink);
    return true;
}

static bool test_sample_bounds(void)
{
    motor_sample_t s = good_sample();
    s.temperature_c = 150.1f;
    CHECK(motor_check_sample(&s) == FAULT_SENSOR);
    s = good_sample();
    s.current_a = MOTOR_CURRENT_LIMIT_A;
    CHECK((motor_check_sample(&s) & FAULT_OVER_CURRENT) != 0u);
    s = good_sample();
    s.bus_v = MOTOR_BUS_MAX_V;
    CHECK(motor_check_sample(&s) == FAULT_NONE);
    return true;
}

static bool test_start_coasting_and_idle_timing(void)
{
    motor_controller_t c;
    ready(&c);
    motor_sample_t s = good_sample();
    s.speed_rpm = MOTOR_STOPPED_RPM + 0.1f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(motor_start(&c) == CTRL_MOTOR_MOVING);
    s.speed_rpm = MOTOR_STOPPED_RPM;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(motor_start(&c) == CTRL_OK);
    CHECK(motor_stop(&c) == CTRL_OK);
    motor_tick(&c, &s, 0u);
    CHECK(motor_start(&c) == CTRL_UNSAFE_SENSORS);
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(motor_start(&c) == CTRL_OK);
    return true;
}

static bool test_sensor_age_and_overspeed(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_start(&c) == CTRL_OK);
    motor_sample_t s = good_sample();
    s.age_ms = MOTOR_SENSOR_MAX_AGE_MS;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_RUNNING);
    ++s.age_ms;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_FAULT && (c.faults & FAULT_SENSOR));
    CHECK(!c.output.enable);
    s = good_sample();
    s.speed_rpm = MOTOR_OVERSPEED_RPM - 0.1f;
    CHECK(motor_check_sample(&s) == FAULT_NONE);
    s.speed_rpm = MOTOR_OVERSPEED_RPM;
    CHECK(motor_check_sample(&s) == FAULT_OVERSPEED);
    return true;
}

static bool test_held_estop_and_reset_hysteresis(void)
{
    motor_controller_t c;
    ready(&c);
    motor_emergency_stop(&c);
    CHECK(motor_reset(&c) == CTRL_UNSAFE_SENSORS);
    motor_release_estop(&c);
    CHECK(c.state == MOTOR_FAULT && !c.output.enable);
    motor_sample_t s = good_sample();
    s.temperature_c = MOTOR_RESET_TEMP_C + 0.1f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(motor_reset(&c) == CTRL_UNSAFE_SENSORS);
    s.temperature_c = MOTOR_RESET_TEMP_C;
    s.current_a = MOTOR_RESET_CURRENT_A + 0.01f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(motor_reset(&c) == CTRL_UNSAFE_SENSORS);
    s.current_a = MOTOR_RESET_CURRENT_A;
    s.speed_rpm = MOTOR_STOPPED_RPM + 0.1f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(motor_reset(&c) == CTRL_UNSAFE_SENSORS);
    s.speed_rpm = MOTOR_STOPPED_RPM;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(motor_reset(&c) == CTRL_OK);
    CHECK(c.faults == 0u && (c.fault_history & FAULT_ESTOP));
    for (int i = 0; i < 49; ++i) motor_tick(&c, &s, MOTOR_TICK_MS);
    s.temperature_c = MOTOR_RESET_TEMP_C + 0.1f;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_RECOVERY && c.recovery_elapsed_ms == 0u);
    s.temperature_c = MOTOR_RESET_TEMP_C;
    for (int i = 0; i < 49; ++i) motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_RECOVERY);
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_IDLE && !c.output.enable);
    return true;
}

typedef struct { motor_output_t output; unsigned writes; } failed_hal_t;
static bool failed_read(void *context, motor_sample_t *sample)
{
    (void)context;
    *sample = good_sample(); /* even a driver that leaves plausible data is rejected */
    return false;
}
static void captured_write(void *context, const motor_output_t *output)
{
    failed_hal_t *h = context;
    h->output = *output;
    ++h->writes;
}
static bool test_hal_acquisition_failure(void)
{
    motor_controller_t c;
    ready(&c);
    CHECK(motor_start(&c) == CTRL_OK);
    failed_hal_t captured = {0};
    motor_hal_t hal = {&captured, failed_read, captured_write};
    CHECK(motor_hal_cycle(&c, &hal, MOTOR_TICK_MS));
    CHECK(c.state == MOTOR_FAULT && (c.faults & FAULT_SENSOR));
    CHECK(captured.writes == 1u && !captured.output.enable && captured.output.duty == 0.0f);
    CHECK(!motor_hal_cycle(&c, NULL, MOTOR_TICK_MS));
    CHECK(c.estop_asserted && !c.output.enable);
    return true;
}

static bool test_binary_frames_and_max_line(void)
{
    uart_parser_t p;
    uart_init(&p);
    const char *prefix = "START";
    for (size_t i = 0; prefix[i]; ++i) uart_feed(&p, prefix[i]);
    uart_feed(&p, '\0');
    CHECK(uart_feed(&p, '\n') == UART_LINE_OVERFLOW);
    for (unsigned i = 0; i < UART_LINE_CAPACITY - 1; ++i)
        CHECK(uart_feed(&p, 'X') == UART_PENDING);
    CHECK(uart_feed(&p, '\n') == UART_LINE_READY);
    CHECK(strlen(p.line) == UART_LINE_CAPACITY - 1u);
    uart_feed(&p, (char)0xff);
    CHECK(uart_feed(&p, '\n') == UART_LINE_OVERFLOW);
    return true;
}

static bool test_parser_adverse_values(void)
{
    motor_app_t a;
    app_init(&a);
    FILE *sink = tmpfile();
    CHECK(sink != NULL);
    const char *bad[] = {"SPEED nan", "SPEED inf", "SPEED 1e99", "SPEED 1000junk",
        "STEP -20", "STEP 0", "STEP 120020", "STEP 99999999999999999",
        "INJECT AGE 1.5", "INJECT CLEAR EXTRA", "ESTOP RELEASE EXTRA", "DIR REV EXTRA"};
    for (size_t i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i) {
        CHECK(!app_execute(&a, bad[i], sink));
        CHECK(a.controller.state == MOTOR_IDLE && a.controller.requested_rpm == 1200.0f);
    }
    fclose(sink);
    return true;
}

static bool test_numeric_threshold_matrix(void)
{
    motor_sample_t s = good_sample();
    s.temperature_c = MOTOR_TEMP_LIMIT_C - 0.001f;
    CHECK(motor_check_sample(&s) == 0u);
    s.temperature_c = MOTOR_TEMP_LIMIT_C;
    CHECK(motor_check_sample(&s) == FAULT_OVER_TEMP);
    s.current_a = MOTOR_CURRENT_LIMIT_A;
    CHECK(motor_check_sample(&s) == (FAULT_OVER_TEMP | FAULT_OVER_CURRENT));
    s = good_sample();
    s.bus_v = MOTOR_BUS_MIN_V;
    CHECK(motor_check_sample(&s) == 0u);
    s.bus_v = MOTOR_BUS_MIN_V - 0.001f;
    CHECK(motor_check_sample(&s) == FAULT_BUS_VOLTAGE);
    s.bus_v = MOTOR_BUS_MAX_V + 0.001f;
    CHECK(motor_check_sample(&s) == FAULT_BUS_VOLTAGE);
    float *fields[] = {&s.speed_rpm, &s.temperature_c, &s.current_a, &s.bus_v};
    for (size_t i = 0; i < 4u; ++i) {
        s = good_sample();
        *fields[i] = INFINITY;
        CHECK(motor_check_sample(&s) == FAULT_SENSOR);
        *fields[i] = NAN;
        CHECK(motor_check_sample(&s) == FAULT_SENSOR);
    }
    return true;
}

static bool test_long_run_load_and_antiwindup(void)
{
    motor_app_t a;
    app_init(&a);
    CHECK(motor_set_speed(&a.controller, 2800.0f) == CTRL_OK);
    CHECK(motor_start(&a.controller) == CTRL_OK);
    a.plant.load = 1.0f; /* unreachable demand: about 1500 rpm at full duty */
    for (int i = 0; i < 3000; ++i) {
        app_step(&a, MOTOR_TICK_MS);
        CHECK(a.controller.state == MOTOR_RUNNING);
        CHECK(isfinite(a.controller.output.duty) && a.controller.output.duty <= 1.0f);
        CHECK(a.controller.integral >= -0.40001f && a.controller.integral <= 0.40001f);
    }
    CHECK(a.controller.output.duty > 0.99f);
    CHECK(motor_set_speed(&a.controller, 500.0f) == CTRL_OK);
    app_step(&a, 20000u);
    CHECK(a.controller.state == MOTOR_RUNNING);
    CHECK(fabsf(a.plant.shaft_rpm - 500.0f) < 100.0f);
    CHECK(motor_stop(&a.controller) == CTRL_OK);
    CHECK(a.controller.integral == 0.0f && a.controller.output.duty == 0.0f);
    return true;
}

static bool test_stall_across_operating_range(void)
{
    const float targets[] = {200.0f, 500.0f, 1800.0f, 2800.0f};
    for (size_t i = 0; i < 4u; ++i) {
        motor_app_t a;
        app_init(&a);
        CHECK(motor_set_speed(&a.controller, targets[i]) == CTRL_OK);
        CHECK(motor_start(&a.controller) == CTRL_OK);
        app_step(&a, 6000u);
        CHECK(a.controller.state == MOTOR_RUNNING); /* no startup false trip */
        a.plant.jammed = true;
        app_step(&a, 1200u);
        CHECK(a.controller.state == MOTOR_FAULT && (a.controller.faults & FAULT_STALL));
        CHECK(!a.controller.output.enable && a.controller.output.duty == 0.0f);
    }
    return true;
}

static bool test_invalid_state_and_fault_history(void)
{
    motor_controller_t c;
    ready(&c);
    c.state = (motor_state_t)999;
    motor_sample_t s = good_sample();
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK(c.state == MOTOR_FAULT && (c.faults & FAULT_INTERNAL));
    s.current_a = MOTOR_CURRENT_LIMIT_A;
    motor_tick(&c, &s, MOTOR_TICK_MS);
    CHECK((c.fault_history & FAULT_OVER_CURRENT) && (c.faults & FAULT_OVER_CURRENT));
    CHECK(!c.output.enable);
    return true;
}

int main(void)
{
    const struct { const char *name; bool (*test)(void); } tests[] = {
        {"initial state requires sensors", test_initial_state},
        {"start stop and zero output", test_start_stop},
        {"unsafe reading prevents start", test_start_rejects_unsafe_reading},
        {"speed bounds and nonfinite", test_speed_limits},
        {"direction locked during run", test_direction_locked_while_running},
        {"PWM slew limited", test_pwm_slew_bound},
        {"overcurrent latches outputs off", test_fault_overcurrent_latches},
        {"overtemperature faults", test_fault_overtemp},
        {"bus voltage faults", test_fault_bus},
        {"sensor dropout faults", test_fault_invalid_sensor},
        {"sensor NaN and negative invalid", test_fault_nan_sensor},
        {"normal plant reaches commanded speed", test_simulated_motor_normal},
        {"mechanical jam generates stall", test_stall_on_mechanical_jam},
        {"reset blocked with unsafe sensors", test_reset_blocked_while_hot},
        {"recovery and no automatic restart", test_reset_requires_cooldown_and_manual_start},
        {"recovery regression faults", test_recovery_invalid_faults_again},
        {"ESTOP immediately disables output", test_estop_synchronous},
        {"missed periodic task causes timing fault", test_missed_tick_fault},
        {"reverse requires shaft stopped", test_reverse_requires_stopped_rotor},
        {"UART line parse and CRLF", test_parser_normal_crlf},
        {"UART overflow is contained", test_parser_overflow_recovers},
        {"injections reset", test_injections_clear},
        {"command interface errors and recovery", test_command_interface},
        {"sample and threshold boundaries", test_sample_bounds},
        {"coasting start and invalid idle scheduling", test_start_coasting_and_idle_timing},
        {"sensor age and overspeed boundary", test_sensor_age_and_overspeed},
        {"held ESTOP and reset hysteresis dwell", test_held_estop_and_reset_hysteresis},
        {"HAL failed acquisition inhibits output", test_hal_acquisition_failure},
        {"binary UART rejection and exact capacity", test_binary_frames_and_max_line},
        {"malformed command values are atomic", test_parser_adverse_values},
        {"threshold and nonfinite input matrix", test_numeric_threshold_matrix},
        {"load saturation antiwindup and recovery", test_long_run_load_and_antiwindup},
        {"stall detection throughout command range", test_stall_across_operating_range},
        {"unknown state and additional fault history", test_invalid_state_and_fault_history}
    };
    unsigned passed = 0u;
    for (size_t i = 0; i < sizeof(tests)/sizeof(tests[0]); ++i) {
        if (tests[i].test()) {
            printf("[PASS] %s\n", tests[i].name);
            ++passed;
        } else {
            printf("[FAIL] %s\n", tests[i].name);
        }
    }
    printf("RESULT: %u/%zu passed\n", passed, sizeof(tests)/sizeof(tests[0]));
    return passed == sizeof(tests)/sizeof(tests[0]) ? 0 : 1;
}
