# DC Motor System Identification and PID Speed Control

Arduino + MATLAB project for AERSP 304 (Penn State): identifying a first-order transfer function for a geared DC motor, then designing and validating a PI speed controller on the hardware.

**Team:** Joe Parker, Robert Danner, Luke Clemens

## Hardware
- Arduino Mega 2560 and an L298N H-bridge, with a 12 V supply
- RK-370SD DC gearmotor with a quadrature encoder (960 counts/rev)

## Part 1: System identification
- `firmware/system_id/system_id.ino` runs 5 open-loop voltage steps (PWM 204 down to 102) plus an optional chirp, and logs time, voltage, and RPM over serial.
- `matlab/motor_system_id.m` fits the DC gain K and the time constant τ (63.2% rise) for each step.
- Identified plant: **G(s) = 31.24 / (0.159 s + 1)** (RPM/V)

## Part 2: Closed-loop control
- `firmware/pid_control/pid_control.ino` is a discrete PID at 10 Hz (Kp = 2, Ki = 15, Kd = 0) tracking a 240 RPM target.
- `matlab/pid_validation.m` compares the measured closed-loop response with the simulated `feedback(C*G,1)` step response, and plots the error.

## Data (`matlab/`)
`motor_data.csv` (step tests), `chirp_data.csv`, `Project2_data.csv` (closed-loop run). Lines starting with `#` are comments.

## Tools
Arduino IDE, MATLAB (Control System Toolbox)
