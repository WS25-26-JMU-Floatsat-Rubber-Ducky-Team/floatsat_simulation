import ctypes
import math
from ctypes import c_float, POINTER, Structure
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

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

        # Virtual actuator mapping
        ("torque_to_rpm", c_float),
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
    c_float,
]
lib.control_step.restype = Vec3

# =========================
# Helpers
# =========================

def quat_normalize(q: Quat):
    n = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
    q.w /= n
    q.x /= n
    q.y /= n
    q.z /= n


def quat_mul(a: Quat, b: Quat) -> Quat:
    return Quat(
        w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    )

def euler_to_quat(roll, pitch, yaw):
    """Convert roll, pitch, yaw (radians) to quaternion."""
    cr = math.cos(roll*0.5)
    sr = math.sin(roll*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)

    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return Quat(w, x, y, z)

def integrate_quat(q: Quat, omega, dt):
    # omega: [wx, wy, wz]
    omega_q = Quat(0.0, omega[0], omega[1], omega[2])
    dq = quat_mul(q, omega_q)

    q.w += 0.5 * dq.w * dt
    q.x += 0.5 * dq.x * dt
    q.y += 0.5 * dq.y * dt
    q.z += 0.5 * dq.z * dt

    quat_normalize(q)


def motor_rpm_to_torque(motor_rpm: Vec3, params: ControlParams):
    """
    Very simple linear model:
      rpm -> torque along wheel axis
    """
    tau = [0.0, 0.0, 0.0]

    for i in range(3):
        rpm = motor_rpm.v[i]
        torque = (rpm / params.max_motor_rpm) * params.max_motor_torque

        tau[0] += torque * params.wheel_axis[i][0]
        tau[1] += torque * params.wheel_axis[i][1]
        tau[2] += torque * params.wheel_axis[i][2]

    return tau

def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0

def setup_plot():
    fig, ax = plt.subplots(2, 1, figsize=(10, 6))
    plt.subplots_adjust(left=0.25, bottom=0.25)

    # Initialize empty lists for plotting
    time_line = []
    yaw_line, pitch_line, roll_line = [], [], []
    yaw_sp_line, pitch_sp_line, roll_sp_line = [], [], []
    omega_line = [[], [], []]  # motor RPM

    # Attitude plot
    att_plot_yaw,   = ax[0].plot(time_line, yaw_line, 'r', label='Yaw')
    att_plot_pitch, = ax[0].plot(time_line, pitch_line, 'g', label='Pitch')
    att_plot_roll,  = ax[0].plot(time_line, roll_line, 'b', label='Roll')

    sp_plot_yaw,   = ax[0].plot(time_line, yaw_sp_line,   'r--', label='Yaw SP')
    sp_plot_pitch, = ax[0].plot(time_line, pitch_sp_line, 'g--', label='Pitch SP')
    sp_plot_roll,  = ax[0].plot(time_line, roll_sp_line,  'b--', label='Roll SP')

    ax[0].set_ylabel('Angle [deg]')
    ax[0].set_ylim(-100, 100)
    ax[0].legend()
    ax[0].grid(True)

    # Motor RPM plot (bottom)
    rpm_plot_lines = [ax[1].plot(time_line, omega_line[i], label=f'Motor {i}')[0] for i in range(3)]
    ax[1].set_ylabel('Motor RPM')
    ax[1].set_xlabel('Time [s]')
    ax[1].set_ylim(-1000, 1000)
    ax[1].legend()
    ax[1].grid(True)

    return (fig, ax,
            att_plot_yaw, att_plot_pitch, att_plot_roll,
            sp_plot_yaw, sp_plot_pitch, sp_plot_roll,
            rpm_plot_lines,
            time_line, yaw_line, pitch_line, roll_line,
            yaw_sp_line, pitch_sp_line, roll_sp_line,
            omega_line)

def setup_sliders(fig, initial_yaw=0.0, initial_pitch=0.0, initial_roll=0.0):
    axcolor = 'lightgoldenrodyellow'
    ax_pitch = plt.axes([0.25, 0.10, 0.65, 0.03], facecolor=axcolor)
    ax_roll  = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)
    ax_yaw   = plt.axes([0.25, 0.05, 0.65, 0.03], facecolor=axcolor)
    ax_omega = plt.axes([0.25, 0.20, 0.65, 0.03], facecolor=axcolor)  # new slider

    pitch_slider = Slider(ax_pitch, 'Pitch', -90.0, 90.0, valinit=initial_pitch)
    roll_slider  = Slider(ax_roll, 'Roll', -90.0, 90.0, valinit=initial_roll)
    yaw_slider   = Slider(ax_yaw, 'Yaw', -90.0, 90.0, valinit=initial_yaw)
    omega_z_slider = Slider(ax_omega, 'Body Z rot', -90.0, 90.0, valinit=0.0)  # deg/s

    return yaw_slider, pitch_slider, roll_slider, omega_z_slider

def setup_3d_visualization(cube_size=0.2):
    fig_3d = plt.figure(figsize=(6,6))
    ax3d = fig_3d.add_subplot(111, projection='3d')
    ax3d.set_xlim([-1,1])
    ax3d.set_ylim([-1,1])
    ax3d.set_zlim([-1,1])
    ax3d.set_xlabel('X')
    ax3d.set_ylabel('Y')
    ax3d.set_zlabel('Z')
    ax3d.set_box_aspect([1,1,1])

    cube_vertices = np.array([
        [-1,-1,-1],[ 1,-1,-1],[ 1, 1,-1],[-1, 1,-1],
        [-1,-1, 1],[ 1,-1, 1],[ 1, 1, 1],[-1, 1, 1],
    ], dtype=float) * cube_size

    cube_edges = [
        (0,1),(1,2),(2,3),(3,0),
        (4,5),(5,6),(6,7),(7,4),
        (0,4),(1,5),(2,6),(3,7)
    ]

    cube_lines = []
    for e in cube_edges:
        line, = ax3d.plot(
            cube_vertices[[e[0], e[1]],0],
            cube_vertices[[e[0], e[1]],1],
            cube_vertices[[e[0], e[1]],2],
            'b'
        )
        cube_lines.append(line)

    # --- Body axes arrows (start along unit axes) ---
    axis_length = cube_size * 2.0
    body_axes = np.eye(3) * axis_length  # X,Y,Z basis vectors

    quivers = [
        ax3d.quiver(0,0,0, body_axes[0,0], body_axes[0,1], body_axes[0,2], color='r'),
        ax3d.quiver(0,0,0, body_axes[1,0], body_axes[1,1], body_axes[1,2], color='g'),
        ax3d.quiver(0,0,0, body_axes[2,0], body_axes[2,1], body_axes[2,2], color='b'),
    ]

    return fig_3d, ax3d, cube_lines, cube_vertices, cube_edges, quivers

def rotate_vectors(vectors, q: Quat):
    qv = np.array([q.w, q.x, q.y, q.z])

    def quat_mul(a, b):
        return np.array([
            a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
            a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
            a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
            a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
        ])

    qc = np.array([qv[0], -qv[1], -qv[2], -qv[3]])

    rotated = []
    for v in vectors:
        vq = np.array([0.0, *v])
        vr = quat_mul(quat_mul(qv, vq), qc)
        rotated.append(vr[1:])
    return np.array(rotated)

def rotate_cube(vertices, q: Quat):
    """
    Rotate vertices by quaternion q
    """
    def quat_mul(a, b):
        w = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
        x = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
        y = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
        z = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
        return np.array([w,x,y,z])
    def quat_conj(qv):
        return np.array([qv[0], -qv[1], -qv[2], -qv[3]])

    qv = np.array([q.w, q.x, q.y, q.z])
    qc = quat_conj(qv)

    rotated = []
    for v in vertices:
        vq = np.array([0.0, *v])
        vr = quat_mul(quat_mul(qv, vq), qc)
        rotated.append(vr[1:])
    return np.array(rotated)


def update_3d_visualization(cube_lines, cube_vertices, cube_edges, quivers, q_current):
    rotated_vertices = rotate_cube(cube_vertices, q_current)

    # Update cube edges
    for i, e in enumerate(cube_edges):
        cube_lines[i].set_data(rotated_vertices[[e[0], e[1]],0],
                               rotated_vertices[[e[0], e[1]],1])
        cube_lines[i].set_3d_properties(rotated_vertices[[e[0], e[1]],2])

    # --- Rotate body axes ---
    axis_length = np.linalg.norm(cube_vertices[0]) * 2
    body_axes = np.eye(3) * axis_length
    rotated_axes = rotate_vectors(body_axes, q_current)

    # Remove old arrows
    for q in quivers:
        q.remove()

    ax = cube_lines[0].axes

    # Draw new arrows
    quivers[0] = ax.quiver(0,0,0, *rotated_axes[0], color='r')
    quivers[1] = ax.quiver(0,0,0, *rotated_axes[1], color='g')
    quivers[2] = ax.quiver(0,0,0, *rotated_axes[2], color='b')

    plt.draw()

def update_plot(att_plots, sp_plots, rpm_plot_lines,
                time_line, yaw_line, pitch_line, roll_line,
                yaw_sp_line, pitch_sp_line, roll_sp_line,
                omega_line, t, angles_deg, sp_angles_deg, motor_rpm,
                max_time_window=30.0):

    yaw, pitch, roll = angles_deg
    yaw_sp, pitch_sp, roll_sp = sp_angles_deg

    # Append new data
    time_line.append(t)
    yaw_line.append(yaw)
    pitch_line.append(pitch)
    roll_line.append(roll)
    yaw_sp_line.append(yaw_sp)
    pitch_sp_line.append(pitch_sp)
    roll_sp_line.append(roll_sp)
    for i in range(3):
        omega_line[i].append(motor_rpm.v[i])

    # Remove old data outside the window
    while time_line and time_line[-1] - time_line[0] > max_time_window:
        time_line.pop(0)
        yaw_line.pop(0)
        pitch_line.pop(0)
        roll_line.pop(0)
        yaw_sp_line.pop(0)
        pitch_sp_line.pop(0)
        roll_sp_line.pop(0)
        for i in range(3):
            omega_line[i].pop(0)

    # Update lines
    att_plots[0].set_data(time_line, yaw_line)
    att_plots[1].set_data(time_line, pitch_line)
    att_plots[2].set_data(time_line, roll_line)
    sp_plots[0].set_data(time_line, yaw_sp_line)
    sp_plots[1].set_data(time_line, pitch_sp_line)
    sp_plots[2].set_data(time_line, roll_sp_line)

    for i in range(3):
        rpm_plot_lines[i].set_data(time_line, omega_line[i])

    # Adjust axes dynamically
    for ax in att_plots[0].axes.figure.axes[:2]:  # first subplot is attitude
        ax.relim()
        ax.autoscale_view()

# =========================
# Main logic
# =========================

def main():
    # -------------------------
    # Control parameters
    # -------------------------

    params = ControlParams()

    params.dt = 0.05  # 500 Hz

    # Outer (angle) PID
    params.angle_kp[:] = (1.0, 1.0, 1.0)
    params.angle_ki[:] = (0.0, 0.0, 0.0)
    params.angle_kd[:] = (1.0, 1.0, 1.0)

    max_ang = deg2rad(10.0)
    params.max_angvel_cmd[:] = (max_ang, max_ang, max_ang)

    # Inner (rate) PID
    params.rate_kp[:] = (6.0, 6.0, 6.0)
    params.rate_ki[:] = (0.0, 0.0, 0.0)
    params.rate_kd[:] = (0.0, 0.0, 0.0)

    # Actuator limits
    params.max_motor_rpm = 900.0
    params.max_motor_torque = 1.0

    # Wheel axis matrix
    params.wheel_axis[0][:] = (0.8165, -0.4083, 0.4083)
    params.wheel_axis[1][:] = (0.0,     0.7071, -0.07071)
    params.wheel_axis[2][:] = (0.5773,  0.5773,  0.5773)

    # Virtual actuator mapping
    params.torque_to_rpm = params.max_motor_rpm / params.max_motor_torque

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
    # Simple plant state
    # -------------------------

    omega = [0.0, 0.0, 0.0]   # body angular velocity [rad/s]

    # Diagonal inertia
    I = [1.0, 1.0, 1.0]
    I_inv = [1.0 / I[0], 1.0 / I[1], 1.0 / I[2]]

    (fig, ax,
     att_plot_yaw, att_plot_pitch, att_plot_roll,
     sp_plot_yaw, sp_plot_pitch, sp_plot_roll,
     rpm_plot_lines,
     time_line, yaw_line, pitch_line, roll_line,
     yaw_sp_line, pitch_sp_line, roll_sp_line,
     omega_line) = setup_plot()
    yaw_slider, pitch_slider, roll_slider, omega_z_slider = setup_sliders(fig, initial_yaw=0.0, initial_pitch=0.0, initial_roll=0.0)
    fig3d, ax3d, cube_lines, cube_vertices, cube_edges, quivers = setup_3d_visualization()

    # Continuous loop with matplotlib interactive mode
    plt.ion()
    plt.show()

    t = 0.0
    dt = params.dt

    print("Running simulation...")

    while True:
        # --- Read setpoints ---
        roll_deg  = roll_slider.val
        pitch_deg = pitch_slider.val
        yaw_deg   = yaw_slider.val
        omega_z_speed = omega_z_slider.val

        roll_rad  = deg2rad(roll_deg)
        pitch_rad = deg2rad(pitch_deg)
        yaw_rad   = deg2rad(yaw_deg)
        omega_z_speed_rad = deg2rad(omega_z_speed)

        q_setpoint = euler_to_quat(roll_rad, pitch_rad, yaw_rad)

        # --- Controller ---
        omega_meas.v[:] = omega

        motor_rpm = lib.control_step(
            ctypes.byref(ctrl_state),
            ctypes.byref(params),
            ctypes.byref(q_current),
            ctypes.byref(q_setpoint),
            ctypes.byref(omega_meas),
            c_float(omega_z_speed_rad),
        )

        # print(motor_rpm.v[0],motor_rpm.v[1], motor_rpm.v[2])

        # --- Plant ---
        tau_from_wheels = [0.0, 0.0, 0.0]
        for i in range(3):
            torque = (motor_rpm.v[i] / params.max_motor_rpm) * params.max_motor_torque
            tau_from_wheels[0] -= torque * params.wheel_axis[i][0]
            tau_from_wheels[1] -= torque * params.wheel_axis[i][1]
            tau_from_wheels[2] -= torque * params.wheel_axis[i][2]

        # Gyroscopic term
        omega_cross_Iomega = [
            omega[1]*I[2]*omega[2] - omega[2]*I[1]*omega[1],
            omega[2]*I[0]*omega[0] - omega[0]*I[2]*omega[2],
            omega[0]*I[1]*omega[1] - omega[1]*I[0]*omega[0]
        ]

        domega = [(tau_from_wheels[i] - omega_cross_Iomega[i])*I_inv[i] for i in range(3)]
        omega = [omega[i] + domega[i]*dt for i in range(3)]
        integrate_quat(q_current, omega, dt)

        # --- Convert current quaternion to Euler angles ---
        w, x, y, z = q_current.w, q_current.x, q_current.y, q_current.z

        # Roll (x-axis)
        roll_current  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        # Pitch (y-axis)
        pitch_current = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
        # Yaw (z-axis)
        yaw_current   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

        angles_deg    = (math.degrees(yaw_current),
                         math.degrees(pitch_current),
                         math.degrees(roll_current))
        sp_angles_deg = (yaw_deg, pitch_deg, roll_deg)

        # --- Update plot ---
        update_plot([att_plot_yaw, att_plot_pitch, att_plot_roll],
                    [sp_plot_yaw, sp_plot_pitch, sp_plot_roll],
                    rpm_plot_lines,
                    time_line, yaw_line, pitch_line, roll_line,
                    yaw_sp_line, pitch_sp_line, roll_sp_line,
                    omega_line,
                    t, angles_deg, sp_angles_deg, motor_rpm)
        update_3d_visualization(cube_lines, cube_vertices, cube_edges, quivers, q_current)

        plt.pause(dt)
        t += dt

if __name__ == "__main__":
    main()
