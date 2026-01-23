#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/* =========================
 * Basic math types
 * ========================= */

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quat_t;

typedef struct {
    float v[3];
} vec3_t;

/* =========================
 * PID state
 * ========================= */

typedef struct {
    float integrator[3];
    float prev_error[3];
} pid_state_t;

/* =========================
 * Control parameters
 * ========================= */

typedef struct {
    /* Outer (angle) loop */
    float angle_kp[3];
    float angle_ki[3];
    float angle_kd[3];
    float max_angvel_cmd[3];     // rad/s clamp

    /* Inner (rate) loop */
    float rate_kp[3];
    float rate_ki[3];
    float rate_kd[3];

    /* Actuator limits */
    float max_motor_rpm;
    float max_motor_torque;

    /* Geometry */
    float wheel_axis[3][3];  // [wheel][xyz]

    /* Timing */
    float dt;

    /* Virtual actuator mapping */
    float torque_to_rpm;     // [RPM / Nm]
} control_params_t;

/* =========================
 * Control runtime state
 * ========================= */

typedef struct {
    pid_state_t angle_pid;
    pid_state_t rate_pid;
} control_state_t;

/* =========================
 * API
 * ========================= */

void control_init(control_state_t *state, const control_params_t *params);

vec3_t control_step(
    control_state_t       *state,
    const control_params_t *params,
    const quat_t          *q_current,
    const quat_t          *q_setpoint,
    const vec3_t          *omega_meas  // Angvel measurement across axes for inner loop propagation
);

#endif /* CONTROL_H */
