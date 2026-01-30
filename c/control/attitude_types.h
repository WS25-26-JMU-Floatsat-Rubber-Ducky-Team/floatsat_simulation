#ifndef ATTITUDE_TYPES_H
#define ATTITUDE_TYPES_H

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quat_t;

typedef struct {
    float v[3];
} vec3_t;

#endif /* ATTITUDE_TYPES_H */
