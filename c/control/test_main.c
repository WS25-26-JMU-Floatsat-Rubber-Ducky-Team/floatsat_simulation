#include <stdio.h>
#include <math.h>

#include "control.h"

/* Helper: degrees to radians */
static float deg2rad(float deg)
{
    return deg * (float)M_PI / 180.0f;
}

int main(void)
{
    /* =========================
     * Control parameters
     * ========================= */

    control_params_t params = {0};

    /* Timing */
    params.dt = 0.01f;  /* 100 Hz */

    /* Outer (angle) PID */
    params.angle_kp[0] = 1.0f;
    params.angle_kp[1] = 1.0f;
    params.angle_kp[2] = 1.0f;

    params.angle_ki[0] = 0.0f;
    params.angle_ki[1] = 0.0f;
    params.angle_ki[2] = 0.0f;

    params.max_angvel_cmd[0] = deg2rad(10.0f);
    params.max_angvel_cmd[1] = deg2rad(10.0f);
    params.max_angvel_cmd[2] = deg2rad(10.0f);

    /* Inner (rate) PID */
    params.rate_kp[0] = 1.0f;
    params.rate_kp[1] = 1.0f;
    params.rate_kp[2] = 1.0f;

    params.rate_ki[0] = 0.5f;
    params.rate_ki[1] = 0.5f;
    params.rate_ki[2] = 0.5f;

    params.rate_kd[0] = 0.0f;
    params.rate_kd[1] = 0.0f;
    params.rate_kd[2] = 0.0f;

    /* Actuator limits */
    params.max_motor_rpm    = 900.0f;
    params.max_motor_torque = 0.2f;

    /* =========================
     * Wheel axis matrix (identity)
     * ========================= */
    /*
     * Wheel 0 → X
     * Wheel 1 → Y
     * Wheel 2 → Z
     */
    params.wheel_axis[0][0] = 0.8165f;
    params.wheel_axis[0][1] = -0.4083f;
    params.wheel_axis[0][2] = 0.4083f;

    params.wheel_axis[1][0] = 0.0f;
    params.wheel_axis[1][1] = 0.7071f;
    params.wheel_axis[1][2] = -0.07071f;

    params.wheel_axis[2][0] = 0.5773f;
    params.wheel_axis[2][1] = 0.5773f;
    params.wheel_axis[2][2] = 0.5773f;

    /* =========================
     * Control state
     * ========================= */

    control_state_t ctrl_state;
    control_init(&ctrl_state, &params);

    /* =========================
     * Example inputs
     * ========================= */

    /* Current attitude: identity quaternion */
    quat_t q_current = {
        .w = 1.0f,
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f
    };

    /* Setpoint: 10 deg yaw */
    float yaw = deg2rad(10.0f);
    quat_t q_setpoint = {
        .w = cosf(yaw * 0.5f),
        .x = 0.0f,
        .y = 0.0f,
        .z = sinf(yaw * 0.5f)
    };

    /* Measured angular velocity (gyro) */
    vec3_t omega_meas = {
        .v = { 0.0f, 0.0f, 0.0f }
    };

    /* Output */
    vec3_t motor_rpm = {0};

    /* =========================
     * Run one control step
     * ========================= */
    motor_rpm = control_step(
        &ctrl_state,
        &params,
        &q_current,
        &q_setpoint,
        &omega_meas
    );

    /* =========================
     * Print result
     * ========================= */

    printf("Motor RPM outputs:\n");
    printf("  Motor 0: %.3f RPM\n", motor_rpm.v[0]);
    printf("  Motor 1: %.3f RPM\n", motor_rpm.v[1]);
    printf("  Motor 2: %.3f RPM\n", motor_rpm.v[2]);

    return 0;
}
