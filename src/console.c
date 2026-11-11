#include "console.h"
#include "uart_protocol.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static bool parse_float(const char *text, float *value)
{
    if (text == NULL) return false;
    char *end;
    errno = 0;
    double parsed = strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0' ||
        !isfinite(parsed) || fabs(parsed) > 1e6) return false;
    *value = (float)parsed;
    return true;
}

static bool parse_duration(const char *text, uint32_t *value)
{
    if (text == NULL || text[0] == '-') return false;
    char *end;
    errno = 0;
    unsigned long parsed = strtoul(text, &end, 10);
    if (errno || end == text || *end != '\0' || parsed == 0ul ||
        parsed > 120000ul || parsed % MOTOR_TICK_MS != 0ul) return false;
    *value = (uint32_t)parsed;
    return true;
}

static void print_faults(uint32_t flags, FILE *out)
{
    if (flags == FAULT_NONE) {
        fputs("NONE", out);
        return;
    }
    const struct { uint32_t flag; const char *text; } names[] = {
        {FAULT_SENSOR, "SENSOR"}, {FAULT_OVER_TEMP, "OVER_TEMP"},
        {FAULT_OVER_CURRENT, "OVER_CURRENT"}, {FAULT_BUS_VOLTAGE, "BUS_VOLTAGE"},
        {FAULT_STALL, "STALL"}, {FAULT_ESTOP, "ESTOP"},
        {FAULT_TIMING, "TIMING"}, {FAULT_OVERSPEED, "OVERSPEED"},
        {FAULT_INTERNAL, "INTERNAL"}
    };
    bool first = true;
    for (size_t i = 0; i < sizeof(names)/sizeof(names[0]); ++i) {
        if ((flags & names[i].flag) != 0u) {
            fprintf(out, "%s%s", first ? "" : "|", names[i].text);
            first = false;
        }
    }
}

static bool sim_read(void *context, motor_sample_t *sample)
{
    motor_app_t *app = context;
    *sample = plant_measure(&app->plant);
    return sample->valid;
}

static void sim_write(void *context, const motor_output_t *output)
{
    motor_app_t *app = context;
    app->applied_output = *output;
}

void app_init(motor_app_t *app)
{
    plant_init(&app->plant);
    motor_init(&app->controller);
    app->hal = (motor_hal_t){app, sim_read, sim_write};
    (void)motor_hal_cycle(&app->controller, &app->hal, MOTOR_TICK_MS);
}

void app_step(motor_app_t *app, uint32_t ms)
{
    for (uint32_t t = 0u; t < ms; t += MOTOR_TICK_MS) {
        /* Commands can inhibit controller outputs synchronously between ticks. */
        sim_write(app, &app->controller.output);
        plant_tick(&app->plant, &app->applied_output, MOTOR_TICK_MS);
        (void)motor_hal_cycle(&app->controller, &app->hal, MOTOR_TICK_MS);
    }
}

void app_status(const motor_app_t *app, FILE *out)
{
    const motor_controller_t *c = &app->controller;
    motor_sample_t sensor = plant_measure(&app->plant);
    fprintf(out, "STATUS t=%llums state=%s direction=%s requested=%.0f rpm ramped=%.0f rpm\n",
            (unsigned long long)c->uptime_ms, motor_state_name(c->state),
            c->direction == DIR_FWD ? "FWD" : "REV", c->requested_rpm,
            c->ramped_rpm);
    fprintf(out, "  output: enable=%d pwm=%.3f; sensor: valid=%d speed=%.1f rpm "
            "temp=%.1f C current=%.2f A bus=%.1f V\n",
            c->output.enable ? 1 : 0, c->output.duty,
            sensor.valid ? 1 : 0, sensor.speed_rpm,
            sensor.temperature_c, sensor.current_a, sensor.bus_v);
    fputs("  latched_faults=", out);
    print_faults(c->faults, out);
    fputc('\n', out);
    fputs("  fault_history=", out);
    print_faults(c->fault_history, out);
    fprintf(out, "; estop_asserted=%d\n", c->estop_asserted ? 1 : 0);
}

static bool result_message(motor_result_t result, FILE *out)
{
    if (result != CTRL_OK) {
        fprintf(out, "ERR %s\n", motor_result_name(result));
        return false;
    }
    fputs("OK\n", out);
    return true;
}

static bool inject(motor_plant_t *p, const char *what, const char *value,
                   const char *extra, FILE *out)
{
    float x;
    if (what != NULL && strcmp(what, "CLEAR") == 0 && value == NULL) {
        plant_clear_injections(p);
    } else if (what != NULL && strcmp(what, "SENSOR") == 0 &&
               value != NULL && extra == NULL &&
               (strcmp(value, "OFF") == 0 || strcmp(value, "ON") == 0)) {
        p->sensor_online = strcmp(value, "ON") == 0;
    } else if (what != NULL && strcmp(what, "JAM") == 0 &&
               value != NULL && extra == NULL &&
               (strcmp(value, "ON") == 0 || strcmp(value, "OFF") == 0)) {
        p->jammed = strcmp(value, "ON") == 0;
    } else if (what != NULL && value != NULL && extra == NULL &&
               parse_float(value, &x)) {
        if (strcmp(what, "TEMP") == 0 && x >= -40.0f && x <= 150.0f) {
            p->force_temperature = true; p->measured_temperature_c = x;
        } else if (strcmp(what, "CURRENT") == 0 && x >= 0 && x <= 100) {
            p->force_current = true; p->measured_current_a = x;
        } else if (strcmp(what, "SPEED") == 0 && x >= 0 && x <= 10000) {
            p->force_speed = true; p->measured_speed_rpm = x;
        } else if (strcmp(what, "BUS") == 0 && x >= 0 && x <= 60) {
            p->force_bus = true; p->measured_bus_v = x;
        } else if (strcmp(what, "LOAD") == 0 && x >= 0 && x <= 1) {
            p->load = x;
        } else if (strcmp(what, "AGE") == 0 && x >= 0 && x <= 10000 && floorf(x) == x) {
            p->sensor_age_ms = (uint32_t)x;
        } else {
            fputs("ERR BAD_INJECTION\n", out);
            return false;
        }
    } else {
        fputs("ERR BAD_INJECTION\n", out);
        return false;
    }
    fputs("OK injection updated; advance STEP to sample it\n", out);
    return true;
}

bool app_execute(motor_app_t *app, const char *command, FILE *out)
{
    if (command == NULL) return false;
    char line[UART_LINE_CAPACITY];
    size_t length = strlen(command);
    if (length >= sizeof(line)) { fputs("ERR LINE_TOO_LONG\n", out); return false; }
    memcpy(line, command, length + 1u);
    char *cmd = strtok(line, " \t");
    if (cmd == NULL || cmd[0] == '#') return true;
    char *arg = strtok(NULL, " \t");
    char *arg2 = strtok(NULL, " \t");
    char *extra = strtok(NULL, " \t");
    if (strcmp(cmd, "HELP") == 0 && arg == NULL) {
        fputs("START | STOP | SPEED <200..2800> | DIR FWD|REV | ESTOP | ESTOP RELEASE | RESET\n"
              "STATUS | STEP <ms, multiple of 20, max 120000> | INJECT CLEAR\n"
              "INJECT TEMP|CURRENT|SPEED|BUS|LOAD|AGE <value> | INJECT SENSOR|JAM ON|OFF\n"
              "All examples use deterministic simulated time; no physical I/O.\n", out);
        return true;
    }
    if (strcmp(cmd, "STATUS") == 0 && arg == NULL) {
        app_status(app, out); return true;
    }
    if (strcmp(cmd, "START") == 0 && arg == NULL)
        return result_message(motor_start(&app->controller), out);
    if (strcmp(cmd, "STOP") == 0 && arg == NULL) {
        motor_result_t result = motor_stop(&app->controller);
        sim_write(app, &app->controller.output);
        return result_message(result, out);
    }
    if (strcmp(cmd, "RESET") == 0 && arg == NULL)
        return result_message(motor_reset(&app->controller), out);
    if (strcmp(cmd, "ESTOP") == 0 && arg == NULL) {
        motor_emergency_stop(&app->controller);
        sim_write(app, &app->controller.output);
        fputs("OK ESTOP latched; output disabled\n", out);
        return true;
    }
    if (strcmp(cmd, "ESTOP") == 0 && arg != NULL &&
        strcmp(arg, "RELEASE") == 0 && arg2 == NULL) {
        motor_release_estop(&app->controller);
        fputs("OK ESTOP released; fault remains latched\n", out);
        return true;
    }
    if (strcmp(cmd, "SPEED") == 0 && arg != NULL && arg2 == NULL) {
        float rpm;
        if (!parse_float(arg, &rpm)) { fputs("ERR BAD_VALUE\n", out); return false; }
        return result_message(motor_set_speed(&app->controller, rpm), out);
    }
    if (strcmp(cmd, "DIR") == 0 && arg != NULL && arg2 == NULL) {
        if (strcmp(arg, "FWD") == 0)
            return result_message(motor_set_direction(&app->controller, DIR_FWD), out);
        if (strcmp(arg, "REV") == 0)
            return result_message(motor_set_direction(&app->controller, DIR_REV), out);
    }
    if (strcmp(cmd, "STEP") == 0 && arg != NULL && arg2 == NULL) {
        uint32_t ms;
        if (!parse_duration(arg, &ms)) {
            fputs("ERR DURATION_MUST_BE_MULTIPLE_OF_20_MS_MAX_120000\n", out);
            return false;
        }
        app_step(app, ms);
        fprintf(out, "OK advanced %u ms\n", (unsigned)ms);
        app_status(app, out);
        return true;
    }
    if (strcmp(cmd, "INJECT") == 0 && arg != NULL)
        return inject(&app->plant, arg, arg2, extra, out);
    fputs("ERR UNKNOWN_COMMAND_OR_ARGUMENTS (use HELP)\n", out);
    return false;
}
