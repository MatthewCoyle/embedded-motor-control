# Safety scope and limitations

This repository implements simulated motor-control and fault-monitoring logic. Its ESTOP, protection and recovery commands describe controller behaviour in software; they are not hardware safety functions or a functional-safety certification.

Setting PWM and enable fields to zero does not prove removal of physical energy. STOP models coasting. Software ESTOP requires the task/host command path to run, cannot interrupt failed CPU execution and is not a physical emergency-stop circuit. Real systems need independently designed inhibition/current limiting/interlocks, a watchdog, validated power-stage sequencing and an appropriate risk assessment.

Sensor-age detection trusts a future HAL to report true acquisition age. It does not detect a plausible frozen/bias value, broken feedback whose driver still says valid, wrong engineering-unit conversion or a stalled scheduler. Output writes are not independently acknowledged. No redundancy, memory integrity, actual ISR timing or hardware fault containment is verified.

Plant equations, thresholds, gains, test traces and graphs are illustrative synthetic values. They cannot predict motor heating, torque, electrical transients, braking energy, EMC behaviour or physical response. Software test passing does not imply equipment suitability.

Before any hardware adaptation: select actual motor/driver/sensors; derive and review limits; identify plant dynamics; implement/validate board peripherals and hardware-off defaults; establish timing budgets with measurement; test ADC/encoder failures, power loss, watchdog, independent stop paths and power-stage faults using appropriate lab equipment. None of that work is included here.

See [test evidence](TEST_RESULTS.md) for the executed software verification.
