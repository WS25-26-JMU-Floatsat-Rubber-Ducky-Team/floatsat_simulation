#include "attitude_estimator.h"
#include <math.h>

#define EPS 1e-6f // Tiny safety constant so you don’t divide by zero/

// Quaternion helpers
static quat_t quat_mul(quat_t a, quat_t b)
{
    quat_t r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

static quat_t quat_norm(quat_t q)
{
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z) + EPS;
    q.w/=n; q.x/=n; q.y/=n; q.z/=n;
    return q;
}

// Euler → quaternion
static quat_t eul2quat(float r, float p, float y)
{
    float cr = cosf(r*0.5f), sr = sinf(r*0.5f);
    float cp = cosf(p*0.5f), sp = sinf(p*0.5f);
    float cy = cosf(y*0.5f), sy = sinf(y*0.5f);

    quat_t q;
    q.w = cy*cp*cr + sy*sp*sr;
    q.x = cy*cp*sr - sy*sp*cr;
    q.y = sy*cp*sr + cy*sp*cr;
    q.z = sy*cp*cr - cy*sp*sr;
    return quat_norm(q);
}

// Acc+Mag → quaternion (absolute attitude)
static quat_t quat_from_accmag(vec3_t acc, vec3_t mag)
{
    float ax=acc.v[0], ay=acc.v[1], az=acc.v[2];

    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az) + EPS);

    float sinr=sinf(roll), cosr=cosf(roll);
    float sinp=sinf(pitch), cosp=cosf(pitch);

    float mx=mag.v[0], my=mag.v[1], mz=mag.v[2];

    float mx2 = mx*cosp + my*sinr*sinp + mz*cosr*sinp;
    float my2 = my*cosr - mz*sinr;

    float yaw = atan2f(-my2, mx2);

    return eul2quat(roll, pitch, yaw);
}

void measurement_init(meas_state_t *s)
{
    s->q.w = 1; s->q.x = s->q.y = s->q.z = 0;
}

measurement_t measurement_update(
    meas_state_t *s,
    const imu_raw_t *raw,
    float dt
)
{
    measurement_t out;

    // Gyro integrate
    float wx=raw->gyro.v[0], wy=raw->gyro.v[1], wz=raw->gyro.v[2];

    quat_t q = s->q;
    quat_t q_dot = {0,
        0.5f*( wx*q.w + wy*q.z - wz*q.y),
        0.5f*(-wx*q.z + wy*q.w + wz*q.x),
        0.5f*( wx*q.y - wy*q.x + wz*q.w)
    };

    quat_t q_gyro = {
        q.w + q_dot.w*dt,
        q.x + q_dot.x*dt,
        q.y + q_dot.y*dt,
        q.z + q_dot.z*dt
    };
    q_gyro = quat_norm(q_gyro);

    // Absolute attitude from acc+mag
    quat_t q_am = quat_from_accmag(raw->acc, raw->mag);

    // Complementary blend (simple LERP + renorm)
    const float alpha_rp  = 0.02f;   // roll/pitch correction

    quat_t q_blend;
    q_blend.w = (1-alpha_rp)*q_gyro.w + alpha_rp*q_am.w;
    q_blend.x = (1-alpha_rp)*q_gyro.x + alpha_rp*q_am.x;
    q_blend.y = (1-alpha_rp)*q_gyro.y + alpha_rp*q_am.y;
    q_blend.z = (1-alpha_rp)*q_gyro.z + alpha_rp*q_am.z;

    s->q = quat_norm(q_blend);

    // Outputs
    out.q = s->q;
    out.omega = raw->gyro; // already rad/s

    return out;
}
