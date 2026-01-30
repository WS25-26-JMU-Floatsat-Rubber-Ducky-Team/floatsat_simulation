import ctypes
import math
from ctypes import c_float, POINTER, Structure
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import signal
import sys

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

        tau[0] -= torque * params.wheel_axis[i][0]
        tau[1] -= torque * params.wheel_axis[i][1]
        tau[2] -= torque * params.wheel_axis[i][2]

    return tau

def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0

# Helper for numpy quaternion multiplication
def quat_np_mul(a, b):
    w = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
    x = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
    y = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
    z = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    return np.array([w, x, y, z])

# =========================
# Simulation plotting & visualization
# =========================

class SimulationPlot:
    """Handles 2D attitude and motor plots."""
    def __init__(self, max_time_window=30.0):
        self.max_time_window = max_time_window

        self.fig, self.ax = plt.subplots(2, 1, figsize=(10, 6))
        plt.subplots_adjust(left=0.25, bottom=0.25)

        # Initialize empty data buffers
        self.time_line = []
        self.yaw_line = []
        self.pitch_line = []
        self.roll_line = []
        self.yaw_sp_line = []
        self.pitch_sp_line = []
        self.roll_sp_line = []
        self.omega_line = [[], [], []]

        # Attitude lines
        self.att_plots = [
            self.ax[0].plot([], [], 'r', label='Yaw')[0],
            self.ax[0].plot([], [], 'g', label='Pitch')[0],
            self.ax[0].plot([], [], 'b', label='Roll')[0],
        ]
        self.sp_plots = [
            self.ax[0].plot([], [], 'r--', label='Yaw SP')[0],
            self.ax[0].plot([], [], 'g--', label='Pitch SP')[0],
            self.ax[0].plot([], [], 'b--', label='Roll SP')[0],
        ]
        self.ax[0].set_ylabel("Angle [deg]")
        self.ax[0].set_ylim(-100, 100)
        self.ax[0].legend()
        self.ax[0].grid(True)

        # Motor RPM lines
        self.rpm_plot_lines = [self.ax[1].plot([], [], label=f'Motor {i}')[0] for i in range(3)]
        self.ax[1].set_ylabel("Motor RPM")
        self.ax[1].set_xlabel("Time [s]")
        self.ax[1].set_ylim(-1000, 1000)
        self.ax[1].legend()
        self.ax[1].grid(True)

    def update(self, t, angles_deg, sp_angles_deg, motor_rpm: Vec3):
        yaw, pitch, roll = angles_deg
        yaw_sp, pitch_sp, roll_sp = sp_angles_deg

        self.time_line.append(t)
        self.yaw_line.append(yaw)
        self.pitch_line.append(pitch)
        self.roll_line.append(roll)
        self.yaw_sp_line.append(yaw_sp)
        self.pitch_sp_line.append(pitch_sp)
        self.roll_sp_line.append(roll_sp)
        for i in range(3):
            self.omega_line[i].append(motor_rpm.v[i])

        # Remove old data outside window
        while self.time_line and self.time_line[-1] - self.time_line[0] > self.max_time_window:
            self.time_line.pop(0)
            self.yaw_line.pop(0)
            self.pitch_line.pop(0)
            self.roll_line.pop(0)
            self.yaw_sp_line.pop(0)
            self.pitch_sp_line.pop(0)
            self.roll_sp_line.pop(0)
            for i in range(3):
                self.omega_line[i].pop(0)

        # Update plots
        for plot, data in zip(self.att_plots, [self.yaw_line, self.pitch_line, self.roll_line]):
            plot.set_data(self.time_line, data)
        for plot, data in zip(self.sp_plots, [self.yaw_sp_line, self.pitch_sp_line, self.roll_sp_line]):
            plot.set_data(self.time_line, data)
        for i, plot in enumerate(self.rpm_plot_lines):
            plot.set_data(self.time_line, self.omega_line[i])

        # Autoscale
        for ax in self.ax:
            ax.relim()
            ax.autoscale_view()


def setup_sliders(fig, initial_yaw=0.0, initial_pitch=0.0, initial_roll=0.0):
    axcolor = 'lightgoldenrodyellow'
    ax_pitch = plt.axes([0.25, 0.10, 0.65, 0.03], facecolor=axcolor)
    ax_roll  = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)
    ax_yaw   = plt.axes([0.25, 0.05, 0.65, 0.03], facecolor=axcolor)
    ax_omega = plt.axes([0.25, 0.20, 0.65, 0.03], facecolor=axcolor)

    pitch_slider = Slider(ax_pitch, 'Pitch', -90.0, 90.0, valinit=initial_pitch)
    roll_slider  = Slider(ax_roll, 'Roll', -90.0, 90.0, valinit=initial_roll)
    yaw_slider   = Slider(ax_yaw, 'Yaw', -90.0, 90.0, valinit=initial_yaw)
    omega_z_slider = Slider(ax_omega, 'Body Z rot', -90.0, 90.0, valinit=0.0)

    return yaw_slider, pitch_slider, roll_slider, omega_z_slider


class Cube3D:
    """3D cube and body axes visualization."""
    def __init__(self, cube_size=0.2):
        self.fig = plt.figure(figsize=(6,6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-1,1])
        self.ax.set_ylim([-1,1])
        self.ax.set_zlim([-1,1])
        self.ax.set_box_aspect([1,1,1])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.vertices = np.array([
            [-1,-1,-1],[ 1,-1,-1],[ 1, 1,-1],[-1, 1,-1],
            [-1,-1, 1],[ 1,-1, 1],[ 1, 1, 1],[-1, 1, 1],
        ], dtype=float) * cube_size

        self.edges = [
            (0,1),(1,2),(2,3),(3,0),
            (4,5),(5,6),(6,7),(7,4),
            (0,4),(1,5),(2,6),(3,7)
        ]
        # Draw cube edges
        self.lines = [self.ax.plot(self.vertices[[e[0], e[1]],0],
                                   self.vertices[[e[0], e[1]],1],
                                   self.vertices[[e[0], e[1]],2], 'b')[0] for e in self.edges]

        # Body axes
        axis_length = cube_size * 2.0
        self.body_axes = np.eye(3) * axis_length
        self.quivers = [
            self.ax.quiver(0,0,0, *self.body_axes[0], color='r'),
            self.ax.quiver(0,0,0, *self.body_axes[1], color='g'),
            self.ax.quiver(0,0,0, *self.body_axes[2], color='b'),
        ]

    @staticmethod
    def rotate_vectors(vectors, q: Quat):
        """Rotate vectors by quaternion."""
        qv = np.array([q.w, q.x, q.y, q.z])
        qc = np.array([q.w, -q.x, -q.y, -q.z])
        rotated = []
        for v in vectors:
            vq = np.array([0.0, *v])
            vr = quat_np_mul(quat_np_mul(qv, vq), qc)
            rotated.append(vr[1:])
        return np.array(rotated)

    @staticmethod
    def rotate_cube(vertices, q: Quat):
        return Cube3D.rotate_vectors(vertices, q)

    def update(self, q: Quat):
        rotated_vertices = self.rotate_cube(self.vertices, q)
        # Update edges
        for i, e in enumerate(self.edges):
            self.lines[i].set_data(rotated_vertices[[e[0], e[1]],0],
                                   rotated_vertices[[e[0], e[1]],1])
            self.lines[i].set_3d_properties(rotated_vertices[[e[0], e[1]],2])
        # Update axes
        rotated_axes = self.rotate_vectors(self.body_axes, q)
        for qv in self.quivers:
            qv.remove()
        self.quivers = [
            self.ax.quiver(0,0,0, *rotated_axes[0], color='r'),
            self.ax.quiver(0,0,0, *rotated_axes[1], color='g'),
            self.ax.quiver(0,0,0, *rotated_axes[2], color='b'),
        ]
        plt.draw()

# =========================
# Main logic
# =========================

def main():
    # -------------------------
    # Initialize control parameters
    # -------------------------
    params = ControlParams()
    params.dt = 0.05  # 20 Hz for simulation

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
    params.wheel_axis[1][:] = (0.0, 0.7071, -0.07071)
    params.wheel_axis[2][:] = (0.5773, 0.5773, 0.5773)

    # Virtual actuator mapping
    params.torque_to_rpm = params.max_motor_rpm / params.max_motor_torque

    # -------------------------
    # Initialize controller state
    # -------------------------
    ctrl_state = ControlState()
    lib.control_init(ctypes.byref(ctrl_state), ctypes.byref(params))

    # -------------------------
    # Initialize simulation state
    # -------------------------
    q_current = Quat(1.0, 0.0, 0.0, 0.0)
    omega = [0.0, 0.0, 0.0]  # Body rates [rad/s]
    I = [1.0, 1.0, 1.0]      # Diagonal inertia
    I_inv = [1.0 / i for i in I]
    omega_meas = Vec3((0.0, 0.0, 0.0))

    # -------------------------
    # Setup visualization
    # -------------------------
    sim_plot = SimulationPlot(max_time_window=30.0)
    yaw_slider, pitch_slider, roll_slider, omega_z_slider = setup_sliders(sim_plot.fig)
    cube3d = Cube3D()

    plt.ion()
    plt.show()

    # -------------------------
    # Simulation loop
    # -------------------------
    t = 0.0
    dt = params.dt
    print("Running simulation...")

    while plt.fignum_exists(sim_plot.fig.number) and plt.fignum_exists(cube3d.fig.number):
        # --- Read user setpoints ---
        roll_deg  = roll_slider.val
        pitch_deg = pitch_slider.val
        yaw_deg   = yaw_slider.val
        omega_z_deg = omega_z_slider.val

        roll_rad  = deg2rad(roll_deg)
        pitch_rad = deg2rad(pitch_deg)
        yaw_rad   = deg2rad(yaw_deg)
        omega_z_rad = deg2rad(omega_z_deg)

        q_setpoint = euler_to_quat(roll_rad, pitch_rad, yaw_rad)

        # --- Controller ---
        omega_meas.v[:] = omega
        motor_rpm = lib.control_step(
            ctypes.byref(ctrl_state),
            ctypes.byref(params),
            ctypes.byref(q_current),
            ctypes.byref(q_setpoint),
            ctypes.byref(omega_meas),
            c_float(omega_z_rad),
        )

        # --- Simple plant integration ---
        tau = motor_rpm_to_torque(motor_rpm, params)
        # Gyroscopic term: omega x (I*omega)
        omega_cross_Iomega = [
            omega[1]*I[2]*omega[2] - omega[2]*I[1]*omega[1],
            omega[2]*I[0]*omega[0] - omega[0]*I[2]*omega[2],
            omega[0]*I[1]*omega[1] - omega[1]*I[0]*omega[0]
        ]
        domega = [(tau[i] - omega_cross_Iomega[i]) * I_inv[i] for i in range(3)]
        omega = [omega[i] + domega[i]*dt for i in range(3)]
        integrate_quat(q_current, omega, dt)

        # --- Current angles for plotting ---
        roll_current  = math.atan2(2*(q_current.w*q_current.x + q_current.y*q_current.z),
                                   1 - 2*(q_current.x**2 + q_current.y**2))
        pitch_current = math.asin(max(-1.0, min(1.0, 2*(q_current.w*q_current.y - q_current.z*q_current.x))))
        yaw_current   = math.atan2(2*(q_current.w*q_current.z + q_current.x*q_current.y),
                                   1 - 2*(q_current.y**2 + q_current.z**2))
        angles_deg = (math.degrees(yaw_current),
                      math.degrees(pitch_current),
                      math.degrees(roll_current))
        sp_angles_deg = (yaw_deg, pitch_deg, roll_deg)

        # --- Update plots ---
        sim_plot.update(t, angles_deg, sp_angles_deg, motor_rpm)
        cube3d.update(q_current)

        plt.pause(dt)
        t += dt

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    finally:
        import matplotlib.pyplot as plt
        plt.ioff()
        plt.close('all')
