#include <stdio.h>
#include <math.h>

#include "attitude_control.h"
#include "attitude_estimator.h"

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

    params.torque_to_rpm = params.max_motor_rpm / params.max_motor_torque;

    /* =========================
     * Control state
     * ========================= */

    control_state_t ctrl_state;
    control_init(&ctrl_state, &params);

    /* =========================
     * Estimator state
     * ========================= */

    meas_state_t meas_state;
    measurement_init(&meas_state);

    /* =========================
     * Example inputs
     * ========================= */

    /* Estimator input */
    imu_raw_t imu = {0};
    measurement_t meas = {0};

    imu.gyro.v[0] = 0.0f;
    imu.gyro.v[1] = 0.0f;
    imu.gyro.v[2] = 0.0f;

    /* Accel sees gravity "down" in body frame */
    imu.acc.v[0] = 0.0f;
    imu.acc.v[1] = 0.0f;
    imu.acc.v[2] = -9.81f;

    /* Magnetic field (arbitrary normalized vector) */
    imu.mag.v[0] = 0.3f;
    imu.mag.v[1] = 0.0f;
    imu.mag.v[2] = 0.5f;

    /* Setpoint: 10 deg yaw */
    float yaw = deg2rad(10.0f);
    quat_t q_setpoint = {
        .w = cosf(yaw * 0.5f),
        .x = 0.0f,
        .y = 0.0f,
        .z = sinf(yaw * 0.5f)
    };

    /* Optional spin command */
    float omega_body_z_cmd = 0.0f;

    /* Output */
    vec3_t motor_rpm = {0};

    /* =========================
     * Run one control step
     * ========================= */
    meas = measurement_update(&meas_state, &imu, params.dt);

    motor_rpm = control_step(
        &ctrl_state,
        &params,
        &meas.q,
        &q_setpoint,
        &meas.omega,
        omega_body_z_cmd
    );

    /* =========================
     * Print result
     * ========================= */
    printf("Estimated quaternion: w=%.4f x=%.4f y=%.4f z=%.4f\n",
           meas.q.w, meas.q.x, meas.q.y, meas.q.z);
    printf("Motor RPM outputs:\n");
    printf("  Motor 0: %.3f RPM\n", motor_rpm.v[0]);
    printf("  Motor 1: %.3f RPM\n", motor_rpm.v[1]);
    printf("  Motor 2: %.3f RPM\n", motor_rpm.v[2]);

    return 0;
}
