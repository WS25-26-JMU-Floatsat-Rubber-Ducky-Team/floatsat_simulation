#include "control.h"
#include <math.h>

/* =========================
 * Small helpers
 * ========================= */

static float clamp(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

static quat_t quat_conj(quat_t q)
{
    quat_t r = { q.w, -q.x, -q.y, -q.z };
    return r;
}

static quat_t quat_mul(quat_t a, quat_t b)
{
    quat_t r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

/* =========================
 * Initialization
 * ========================= */

void control_init(control_state_t *state, const control_params_t *params)
{
    (void)params;

    for (int i = 0; i < 3; i++) {
        state->angle_pid.integrator[i] = 0.0f;
        state->angle_pid.prev_error[i] = 0.0f;

        state->rate_pid.integrator[i]  = 0.0f;
        state->rate_pid.prev_error[i]  = 0.0f;
    }
}


static void angle_controller(
    pid_state_t *pid,
    const control_params_t *p,
    const quat_t *q_current,
    const quat_t *q_setpoint,
    vec3_t *omega_cmd
)
{
    quat_t qc = quat_conj(*q_current);
    quat_t q_err = quat_mul(qc, *q_setpoint);

    /* shortest rotation */
    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    float rot_err[3] = {
        2.0f * q_err.x,
        2.0f * q_err.y,
        2.0f * q_err.z
    };

    for (int i = 0; i < 3; i++) {

        float p_term = p->angle_kp[i] * rot_err[i];
        float i_term = pid->integrator[i];

        float omega_unsat = p_term + i_term;

        float omega_sat = clamp(
            omega_unsat,
            -p->max_angvel_cmd[i],
             p->max_angvel_cmd[i]
        );

        /* Anti-windup: conditional integration */
        float err = rot_err[i];

        int saturated = (omega_unsat != omega_sat);

        if (!saturated ||
            (omega_unsat >  p->max_angvel_cmd[i] && err < 0.0f) ||
            (omega_unsat < -p->max_angvel_cmd[i] && err > 0.0f)) {

            pid->integrator[i] += err * p->angle_ki[i] * p->dt;
        }

        omega_cmd->v[i] = omega_sat;
    }
}

static void rate_controller(
    pid_state_t *pid,
    const control_params_t *p,
    const vec3_t *omega_cmd,
    const vec3_t *omega_meas,
    vec3_t *tau_body
)
{
    for (int i = 0; i < 3; i++) {
        float err = omega_cmd->v[i] - omega_meas->v[i];

        float p_term = p->rate_kp[i] * err;
        float d_term = p->rate_kd[i] *
                       (err - pid->prev_error[i]) / p->dt;

        float tau_unsat =
            p_term +
            pid->integrator[i] +
            d_term;

        float tau_sat = clamp(
            tau_unsat,
            -p->max_motor_torque,
             p->max_motor_torque
        );

        /* Anti-windup */
        int saturated = (tau_unsat != tau_sat);

        if (!saturated ||
            (tau_unsat >  p->max_motor_torque && err < 0.0f) ||
            (tau_unsat < -p->max_motor_torque && err > 0.0f)) {

            pid->integrator[i] += err * p->rate_ki[i] * p->dt;
        }

        pid->prev_error[i] = err;
        tau_body->v[i] = tau_sat;
    }
}

static void allocate_torque(
    const control_params_t *p,
    vec3_t *tau_body,
    float motor_tau[3]
)
{
    float motor_tau_pred[3];
    float scale = 1.0f;

    /* --- Predict wheel torques (no clamping) --- */
    for (int i = 0; i < 3; i++) {
        motor_tau_pred[i] =
            -(p->wheel_axis[i][0] * tau_body->v[0] +
              p->wheel_axis[i][1] * tau_body->v[1] +
              p->wheel_axis[i][2] * tau_body->v[2]);

        float ratio = fabsf(motor_tau_pred[i]) / p->max_motor_torque;
        if (ratio > scale) {
            scale = ratio;
        }
    }

    /* --- Scale body torque if needed --- */
    if (scale > 1.0f) {
        float inv = 1.0f / scale;
        for (int i = 0; i < 3; i++) {
            tau_body->v[i] *= inv;
            motor_tau_pred[i] *= inv;
        }
    }

    /* --- Final motor torques (with hard safety clamp) --- */
    for (int i = 0; i < 3; i++) {
        motor_tau[i] = clamp(
            motor_tau_pred[i],
            -p->max_motor_torque,
             p->max_motor_torque
        );
    }
}

vec3_t control_step(
    control_state_t        *state,
    const control_params_t *params,
    const quat_t           *q_current,
    const quat_t           *q_setpoint,
    const vec3_t           *omega_meas
)
{
    vec3_t omega_cmd;
    vec3_t tau_body;
    float motor_tau[3];
    vec3_t motor_rpm;

    angle_controller(
        &state->angle_pid,
        params,
        q_current,
        q_setpoint,
        &omega_cmd
    );

    rate_controller(
        &state->rate_pid,
        params,
        &omega_cmd,
        omega_meas,
        &tau_body
    );

    allocate_torque(
        params,
        &tau_body,
        motor_tau
    );

    for (int i = 0; i < 3; i++) {
        float rpm = motor_tau[i] * params->torque_to_rpm;
        motor_rpm.v[i] = clamp(
            rpm,
            -params->max_motor_rpm,
             params->max_motor_rpm
        );
    }

    return motor_rpm;
}
