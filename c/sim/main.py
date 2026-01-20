import ctypes
import math
from ctypes import c_float, POINTER, Structure

# =========================
# Load shared library
# =========================

lib = ctypes.CDLL("../control/libcontrol.so")

# =========================
# ctypes struct definitions
# =========================

class Quat(Structure):
    _fields_ = [
        ("w", c_float),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
    ]


class Vec3(Structure):
    _fields_ = [
        ("v", c_float * 3),
    ]


class PIDState(Structure):
    _fields_ = [
        ("integrator", c_float * 3),
        ("prev_error", c_float * 3),
    ]


class ControlParams(Structure):
    _fields_ = [
        # Outer (angle) loop
        ("angle_kp", c_float * 3),
        ("angle_ki", c_float * 3),
        ("angle_kd", c_float * 3),
        ("max_angvel_cmd", c_float * 3),

        # Inner (rate) loop
        ("rate_kp", c_float * 3),
        ("rate_ki", c_float * 3),
        ("rate_kd", c_float * 3),

        # Actuator limits
        ("max_motor_rpm", c_float),
        ("max_motor_torque", c_float),

        # Geometry
        ("wheel_axis", (c_float * 3) * 3),

        # Timing
        ("dt", c_float),
    ]


class ControlState(Structure):
    _fields_ = [
        ("angle_pid", PIDState),
        ("rate_pid", PIDState),
    ]


# =========================
# Function prototypes
# =========================

lib.control_init.argtypes = [
    POINTER(ControlState),
    POINTER(ControlParams),
]
lib.control_init.restype = None

lib.control_step.argtypes = [
    POINTER(ControlState),
    POINTER(ControlParams),
    POINTER(Quat),
    POINTER(Quat),
    POINTER(Vec3),
]
lib.control_step.restype = Vec3


# =========================
# Helper
# =========================

def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


# =========================
# Main logic (ported from C)
# =========================

def main():
    # -------------------------
    # Control parameters
    # -------------------------

    params = ControlParams()

    params.dt = 0.01  # 100 Hz

    # Outer (angle) PID
    params.angle_kp[:] = (1.0, 1.0, 1.0)
    params.angle_ki[:] = (0.0, 0.0, 0.0)
    params.angle_kd[:] = (0.0, 0.0, 0.0)

    max_ang = deg2rad(10.0)
    params.max_angvel_cmd[:] = (max_ang, max_ang, max_ang)

    # Inner (rate) PID
    params.rate_kp[:] = (1.0, 1.0, 1.0)
    params.rate_ki[:] = (0.5, 0.5, 0.5)
    params.rate_kd[:] = (0.0, 0.0, 0.0)

    # Actuator limits
    params.max_motor_rpm = 900.0
    params.max_motor_torque = 0.2

    # Wheel axis matrix
    params.wheel_axis[0][:] = (0.8165, -0.4083, 0.4083)
    params.wheel_axis[1][:] = (0.0,     0.7071, -0.07071)
    params.wheel_axis[2][:] = (0.5773,  0.5773,  0.5773)

    # -------------------------
    # Control state
    # -------------------------

    ctrl_state = ControlState()
    lib.control_init(ctypes.byref(ctrl_state), ctypes.byref(params))

    # -------------------------
    # Inputs
    # -------------------------

    q_current = Quat(
        w=1.0,
        x=0.0,
        y=0.0,
        z=0.0,
    )

    yaw = deg2rad(10.0)
    q_setpoint = Quat(
        w=math.cos(yaw * 0.5),
        x=0.0,
        y=0.0,
        z=math.sin(yaw * 0.5),
    )

    omega_meas = Vec3((0.0, 0.0, 0.0))

    # -------------------------
    # Run one control step
    # -------------------------

    motor_rpm = lib.control_step(
        ctypes.byref(ctrl_state),
        ctypes.byref(params),
        ctypes.byref(q_current),
        ctypes.byref(q_setpoint),
        ctypes.byref(omega_meas),
    )

    # -------------------------
    # Print result
    # -------------------------

    print("Motor RPM outputs:")
    print(f"  Motor 0: {motor_rpm.v[0]:.3f} RPM")
    print(f"  Motor 1: {motor_rpm.v[1]:.3f} RPM")
    print(f"  Motor 2: {motor_rpm.v[2]:.3f} RPM")


if __name__ == "__main__":
    main()
