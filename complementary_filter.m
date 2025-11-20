addpath("utils");

function y = clamp(x, lo, hi)
  if x > hi
    y = hi;
  elseif x < lo
    y = lo;
  else
    y = x;
  end
endfunction

function acc_body = true_accel_from_euler(roll, pitch, yaw, G)
  % Compute accelerometer reading in body frame due to gravity only.
  % With our sign convention a level vehicle sees [0,0,G].
  % Standard mapping:
  ax = -G * sin(pitch);
  ay =  G * sin(roll) * cos(pitch);
  az =  G * cos(roll) * cos(pitch);
  acc_body = [ax, ay, az];
endfunction

%% ---------------- Complementary Filter Logic ---------------
function [roll_f, pitch_f, yaw_f, qf] = comp_step(roll_prev, pitch_prev, yaw_prev, gyr, acc, params)
  % Single complementary-filter update step performing:
  %  - integrate gyro to get angle prediction
  %  - compute accel-based roll/pitch
  %  - blend with ALPHA
  %  - return fused euler and quaternion (normalized)
  DT = params.DT; ALPHA = params.ALPHA; EPS = params.EPS;
  ix = params.IDX_X; iy = params.IDX_Y; iz = params.IDX_Z;

  % 1) Gyro integration (predict)
  roll_gyro  = roll_prev  + gyr(ix) * DT;
  pitch_gyro = pitch_prev + gyr(iy) * DT;
  yaw_gyro   = yaw_prev   + gyr(iz) * DT;   % needs magnetometer for correction

  % 2) Accelerometer-based angles (measurement)
  ax = acc(ix); ay = acc(iy); az = acc(iz);
  % Protect sqrt against tiny negative due to roundoff with EPS
  denom = sqrt(ay*ay + az*az + EPS);
  roll_acc  = atan2(ay, az);
  pitch_acc = atan2(-ax, denom);

  % 3) Complementary fusion (low-pass accel, high-pass gyro)
  roll_f  = ALPHA * roll_gyro  + (1 - ALPHA) * roll_acc;
  pitch_f = ALPHA * pitch_gyro + (1 - ALPHA) * pitch_acc;
  yaw_f   = yaw_gyro;                       % without mag: only integrate gyro

  % 4) Wrap angles to [-pi, pi] (helps numerical stability and porting)
  roll_f  = wrap_angle(roll_f);
  pitch_f = wrap_angle(pitch_f);
  yaw_f   = wrap_angle(yaw_f);

  % 5) Convert to quaternion and normalize
  qf = eul2quat(roll_f, pitch_f, yaw_f);
  qf = normalize_quat(qf);
endfunction

%% ---------------- Configuration / Constants ----------------
% Simulation parameters
SAMPLE_RATE_HZ = 100;            % samples per second
DT = 1.0 / SAMPLE_RATE_HZ;       % seconds per sample
SIM_DURATION_S = 4.0;            % simulation length in seconds
N = round(SIM_DURATION_S * SAMPLE_RATE_HZ);

% Physical constants
G = 9.81;                        % gravity magnitude (m/s^2)
EPS = 1e-8;                      % small epsilon for numeric safety

% Sensor axis indices (helps when porting later)
IDX_X = 1; IDX_Y = 2; IDX_Z = 3;

% Complementary filter tuning (use time-constant TAU, not a magic alpha)
% TAU is the time constant (seconds) of the low-pass applied to accel-based angle.
TAU = 0.5;                       % seconds (try 0.5..2.0)
ALPHA = TAU / (TAU + DT);        % alpha in [0,1] computed from TAU

% Initial yaw used when no magnetometer exists
INITIAL_YAW = 0.0;               % radians

% PID tuning (per-axis)
PID.Kp = [4.0, 4.0, 4.0];
PID.Ki = [1.0, 1.0, 1.0];
PID.Kd = [0.1, 0.1, 0.1];
PID.tau_D = 0.02;   % derivative filter time constant (s)
PID.integrator_min = -0.5;    % will be scaled relative to actuator limits
PID.integrator_max =  0.5;

% Actuator model
ACTUATOR_MAX_ANGVEL_DEG = 60;                 % deg/s
ACTUATOR_MAX_ANGVEL = deg2rad(ACTUATOR_MAX_ANGVEL_DEG); % rad/s
ACTUATOR_MIN_ANGVEL = -ACTUATOR_MAX_ANGVEL;
ACTUATOR_TIMECONST = 0.05;    % s (0 => ideal actuator)

% Pack params for comp_step
params.DT = DT;
params.ALPHA = ALPHA;
params.EPS = EPS;
params.IDX_X = IDX_X;
params.IDX_Y = IDX_Y;
params.IDX_Z = IDX_Z;

%% ---------------- Setpoint trajectory (radians) ----------------
% Step at t = 1s
setpoint = zeros(N,3);
for k = 1:N
  if ( (k-1)*DT >= 1.0 )
    setpoint(k, IDX_X) = deg2rad(10);   % roll +10 deg
    setpoint(k, IDX_Y) = deg2rad(-5);   % pitch -5 deg
    setpoint(k, IDX_Z) = deg2rad(20);   % yaw +20 deg
  endif
endfor

%% ---------------- State initialization ----------------
% True plant state (what the world really is)
angle_true = zeros(N,3);       % [roll pitch yaw] (rad)
angvel_true = zeros(N,3);      % true angular velocity (rad/s)
angvel_act_state = zeros(1,3); % actuator internal state (for lag)

% Filter state (what the filter estimates)
roll_f = zeros(N,1);
pitch_f = zeros(N,1);
yaw_f = zeros(N,1);
Qf = zeros(N,4);               % fused quaternion [w x y z]

% Sensor arrays (simulated)
gyr_meas = zeros(N,3);
acc_meas = zeros(N,3);

% PID persistent state
pid_state.integrator = [0,0,0];
pid_state.prev_error = [0,0,0];
pid_state.prev_derivative = [0,0,0];

% Command logs
cmd_angvel = zeros(N,3);       % PID raw command (before sat)
cmd_after_act = zeros(N,3);    % after actuator dynamics & sat

%% ---------------- Initial conditions ----------------
% Start level and aligned
angle_true(1,:) = [0, 0, INITIAL_YAW];
angvel_true(1,:) = [0, 0, 0];

% Initial sensor samples from true state  (gyro and accel)
gyr_meas(1,:) = angvel_true(1,:);
acc_meas(1,:) = true_accel_from_euler(angle_true(1,1), angle_true(1,2), angle_true(1,3), G);

% Initial filter estimate from accel (and zero gyro)
roll_f(1)  = atan2(acc_meas(1,2), acc_meas(1,3));
pitch_f(1) = atan2(-acc_meas(1,1), sqrt(acc_meas(1,2)^2 + acc_meas(1,3)^2 + EPS));
yaw_f(1)   = INITIAL_YAW;
Qf(1,:) = eul2quat(roll_f(1), pitch_f(1), yaw_f(1));
Qf(1,:) = normalize_quat(Qf(1,:));

% Scale integrator bounds to actuator (use sensible default fraction)
PID.integrator_min = PID.integrator_min * ACTUATOR_MAX_ANGVEL;
PID.integrator_max = PID.integrator_max * ACTUATOR_MAX_ANGVEL;

%% ---------------- Main loop ----------------
for k = 2:N
  % 1) sensors read the current true state (we simulate ideal sensors here)
  gyr_meas(k,:) = angvel_true(k-1,:);   % gyro measures true angular velocity
  acc_meas(k,:) = true_accel_from_euler(angle_true(k-1,1), angle_true(k-1,2), angle_true(k-1,3), G);

  % 2) complementary filter uses sensor samples to produce fused angles
  [rf, pf, yf, qf] = comp_step(roll_f(k-1), pitch_f(k-1), yaw_f(k-1), gyr_meas(k,:), acc_meas(k,:), params);
  roll_f(k) = rf; pitch_f(k) = pf; yaw_f(k) = yf; Qf(k,:) = qf;

  % 3) PID controller uses *filtered* angles (not true angles)
  sp = setpoint(k,:);
  meas = [rf, pf, yf];

  for axis = 1:3
    % error (wrap)
    err = wrap_angle(sp(axis) - meas(axis));

    % P term
    P = PID.Kp(axis) * err;

    % I term (trapezoidal not needed here; simple rectangular integration suffices)
    pid_state.integrator(axis) = pid_state.integrator(axis) + err * DT * PID.Ki(axis);
    % anti-windup clamp
    if pid_state.integrator(axis) > PID.integrator_max
      pid_state.integrator(axis) = PID.integrator_max;
    elseif pid_state.integrator(axis) < PID.integrator_min
      pid_state.integrator(axis) = PID.integrator_min;
    endif
    I = pid_state.integrator(axis);

    % D term (filtered derivative)
    derivative_raw = (err - pid_state.prev_error(axis)) / DT;
    tau = PID.tau_D;
    D_filtered = (tau * pid_state.prev_derivative(axis) + DT * derivative_raw) / (tau + DT);
    pid_state.prev_derivative(axis) = D_filtered;
    pid_state.prev_error(axis) = err;
    D = PID.Kd(axis) * D_filtered;

    % PID output interpreted as desired angular velocity [rad/s]
    u = P + I + D;
    cmd_angvel(k, axis) = u;
  endfor

  % 4) Actuator dynamics: apply saturation and first-order lag
  for axis = 1:3
    u_sat = clamp(cmd_angvel(k,axis), ACTUATOR_MIN_ANGVEL, ACTUATOR_MAX_ANGVEL);

    if ACTUATOR_TIMECONST <= EPS
      angvel_act_state(axis) = u_sat;
    else
      alpha_act = DT / (ACTUATOR_TIMECONST + DT);  % discrete alpha for first-order
      angvel_act_state(axis) = angvel_act_state(axis) + alpha_act * (u_sat - angvel_act_state(axis));
    endif

    cmd_after_act(k, axis) = angvel_act_state(axis);
    angvel_true(k, axis) = angvel_act_state(axis);  % assume actuator directly produces true angvel
  endfor

  % 5) plant integration (true angles updated by true angular velocity)
  angle_true(k, :) = angle_true(k-1, :) + angvel_true(k, :) * DT;
  angle_true(k,1) = wrap_angle(angle_true(k,1));
  angle_true(k,2) = wrap_angle(angle_true(k,2));
  angle_true(k,3) = wrap_angle(angle_true(k,3));
endfor

%% ---------------- Results / example prints ----------------
fprintf('Final true angles (deg) [roll pitch yaw]:\n');
disp(rad2deg(angle_true(end,:)));

fprintf('Final filtered angles (deg) [roll pitch yaw]:\n');
disp(rad2deg([roll_f(end), pitch_f(end), yaw_f(end)]));

%% ---------------- Plotting: setpoint vs true vs filtered ----------------
% Time vector
t = (0:(N-1))' * DT;

% Convert to degrees for plotting
deg = @(x) (180/pi) * x;
setpoint_deg = deg(setpoint);
angle_true_deg = deg(angle_true);
angle_filt_deg = deg([roll_f, pitch_f, yaw_f]);

% 1) Angles: roll / pitch / yaw in one figure with three subplots
figure('Name','Angles: Setpoint vs True vs Filtered','NumberTitle','off');

subplot(3,1,1);
plot(t, setpoint_deg(:,IDX_X), '--', 'LineWidth', 1.2); hold on;
plot(t, angle_true_deg(:,IDX_X), '-', 'LineWidth', 1.0);
plot(t, angle_filt_deg(:,IDX_X), ':', 'LineWidth', 1.2);
grid on;
ylabel('Roll (deg)');
legend('setpoint','true','filtered','Location','NorthEast');
title('Roll');

subplot(3,1,2);
plot(t, setpoint_deg(:,IDX_Y), '--', 'LineWidth', 1.2); hold on;
plot(t, angle_true_deg(:,IDX_Y), '-', 'LineWidth', 1.0);
plot(t, angle_filt_deg(:,IDX_Y), ':', 'LineWidth', 1.2);
grid on;
ylabel('Pitch (deg)');
legend('setpoint','true','filtered','Location','NorthEast');
title('Pitch');

subplot(3,1,3);
plot(t, setpoint_deg(:,IDX_Z), '--', 'LineWidth', 1.2); hold on;
plot(t, angle_true_deg(:,IDX_Z), '-', 'LineWidth', 1.0);
plot(t, angle_filt_deg(:,IDX_Z), ':', 'LineWidth', 1.2);
grid on;
ylabel('Yaw (deg)');
xlabel('Time (s)');
legend('setpoint','true','filtered','Location','NorthEast');
title('Yaw');


% 2) Angular velocity commands (deg/s)
figure('Name','Angular velocity commands','NumberTitle','off');
cmd_before_deg = deg(cmd_angvel);       % PID output before saturation/actuator
cmd_after_deg = deg(cmd_after_act);     % after actuator dynamics & saturation
angvel_true_deg = deg(angvel_true);     % actual measured angular velocity

subplot(3,1,1);
plot(t, cmd_before_deg(:,IDX_X), '--', 'LineWidth', 1.0); hold on;
plot(t, cmd_after_deg(:,IDX_X), '-', 'LineWidth', 1.0);
plot(t, angvel_true_deg(:,IDX_X), ':', 'LineWidth', 1.0);
grid on;
ylabel('Roll \omega (deg/s)');
legend('cmd raw','cmd after act','true \omega','Location','NorthEast');
title('Roll angular velocity');

subplot(3,1,2);
plot(t, cmd_before_deg(:,IDX_Y), '--', 'LineWidth', 1.0); hold on;
plot(t, cmd_after_deg(:,IDX_Y), '-', 'LineWidth', 1.0);
plot(t, angvel_true_deg(:,IDX_Y), ':', 'LineWidth', 1.0);
grid on;
ylabel('Pitch \omega (deg/s)');
legend('cmd raw','cmd after act','true \omega','Location','NorthEast');
title('Pitch angular velocity');

subplot(3,1,3);
plot(t, cmd_before_deg(:,IDX_Z), '--', 'LineWidth', 1.0); hold on;
plot(t, cmd_after_deg(:,IDX_Z), '-', 'LineWidth', 1.0);
plot(t, angvel_true_deg(:,IDX_Z), ':', 'LineWidth', 1.0);
grid on;
ylabel('Yaw \omega (deg/s)');
xlabel('Time (s)');
legend('cmd raw','cmd after act','true \omega','Location','NorthEast');
title('Yaw angular velocity');

pause()
