#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "attitude_types.h"

//Raw IMU readings
typedef struct {
    vec3_t gyro;   // rad/s body
    vec3_t acc;    // m/s^2 body
    vec3_t mag;    // normalized body
} imu_raw_t;

typedef struct {
    quat_t q;      // estimated attitude
    vec3_t omega;  // angular velocity (rad/s)
} measurement_t;

typedef struct {
    quat_t q;      // filter state
} meas_state_t;

void measurement_init(meas_state_t *s);

measurement_t measurement_update(
    meas_state_t *s,
    const imu_raw_t *raw,
    float dt
);

#endif /* ATTITUDE_ESTIMATOR_H */
