addpath("utils");

function main()
  % High-level entry: setup, simulate, plot
  params = setup_params();
  state  = setup_state(params);
  [state, params] = run_simulation(state, params);
  plot_results(state, params);
  plot_3d_floatsat(state, params);
  pause();
endfunction

% -------------------------------------------------------------------------
% Params and initialization
% -------------------------------------------------------------------------
function params = setup_params()
  % Simulation params
  params.SAMPLE_RATE_HZ = 100;
  params.DT = 1.0 / params.SAMPLE_RATE_HZ;
  params.SIM_DURATION_S = 30.0;
  params.N = round(params.SIM_DURATION_S * params.SAMPLE_RATE_HZ);

  % Physical constants
  params.G = 9.81;
  params.EPS = 1e-8;

  % Axis indices
  params.IDX_X = 1; params.IDX_Y = 2; params.IDX_Z = 3;

  % Complementary filter tuning (time constant -> alpha)
  params.TAU = 0.5;
  params.ALPHA = params.TAU / (params.TAU + params.DT);

  % Initial yaw
  params.INITIAL_YAW = 0.0;

  % PID tuning (per-axis)
  params.PID.Kp = [1.5, 1.5, 1.5];
  params.PID.Ki = [3, 3, 3];
  params.PID.Kd = [5, 5, 5];
  params.PID.tau_D = 0.02;
  params.PID.integrator_min_frac = -0.5;  % fraction of actuator max (scaled later)
  params.PID.integrator_max_frac =  0.5;

  % Actuator (placeholder: currently maps commanded angvel -> true angvel)
  params.ACTUATOR_MAX_ANGVEL_DEG = 60;
  params.ACTUATOR_MAX_ANGVEL = deg2rad(params.ACTUATOR_MAX_ANGVEL_DEG);
  params.ACTUATOR_MIN_ANGVEL = -params.ACTUATOR_MAX_ANGVEL;
  params.ACTUATOR_TIMECONST = 0.05;

  % Setpoint: change at 1 second
  params.SETPOINT_STEP_TIME = 1.0;
  params.SETPOINT_DEG = [-10, 10, 40]; % [roll, pitch, yaw] in degrees after step

  % Flywheel actuator parameters
  % per-wheel rotational inertia (kg·m^2). Measure or compute from geometry.
  params.FLYWHEEL.Jw = [0.01, 0.01, 0.01];
  % maximum wheel spin rate (rad/s)
  params.FLYWHEEL.wheel_angvel_max = [rpm2rads(900), rpm2rads(900), rpm2rads(900)]; % rad/s
  % motor torque limits
  params.FLYWHEEL.torque_max = [0.2, 0.2, 0.2]; % N·m (example values)

  azimuth_deg = [0, 120, 240];   % wheel azimuths in XY plane
  azimuth_rad = deg2rad(azimuth_deg);

  r_mount = 0.0633;   % distance from body center in XY plane
  z_offset = -0.03022; % slightly below body center along Z

  wheel_pos = zeros(3,3); % 3 wheels x 3 components
  wheel_axis = zeros(3,3);

  for i = 1:3
      az = azimuth_rad(i);
      wheel_pos(i,1) = r_mount * cos(az);  % X
      wheel_pos(i,2) = r_mount * sin(az);  % Y
      wheel_pos(i,3) = z_offset;            % Z
  end

  for i = 1:3
      vec = -wheel_pos(i,:);  % vector from wheel to center
      wheel_axis(i,:) = vec / norm(vec);  % normalize
  end

  params.WHEEL_POS  = wheel_pos;
  params.WHEEL_AXIS = wheel_axis;

  % params.WHEEL_POS = [ 0.063, 0, 0;
  %                      0, 0.063, 0;
  %                      0, 0, -0.063 ];
  % params.WHEEL_AXIS = [ 1, 0, 0;
  %                       0, 1, 0;
  %                       0, 0, 1 ];

  params.I_body = [ 1, 0, 0;
                    0, 1, 0;
                    0, 0, 1 ];  % kg·m^2
  params.I_body_inv = inv(params.I_body);

  % Make params easier to pass to comp_step
  params.comp.DT = params.DT;
  params.comp.ALPHA = params.ALPHA;
  params.comp.EPS = params.EPS;
  params.comp.IDX_X = params.IDX_X;
  params.comp.IDX_Y = params.IDX_Y;
  params.comp.IDX_Z = params.IDX_Z;
endfunction

function state = setup_state(params)
  N = params.N;
  IDX_X = params.IDX_X; IDX_Y = params.IDX_Y; IDX_Z = params.IDX_Z;
  DT = params.DT;

  % Preallocate logs & state
  state.t = (0:(N-1))' * DT;
  state.setpoint = zeros(N,3);
  for k = 1:N
    if ( (k-1)*DT >= params.SETPOINT_STEP_TIME )
      state.setpoint(k,:) = deg2rad(params.SETPOINT_DEG);  % [roll,pitch,yaw] in rad
    end
  endfor

  state.angle_true = zeros(N,3);
  state.angvel_true = zeros(N,3);
  state.angvel_act_state = zeros(1,3);

  state.roll_f = zeros(N,1); state.pitch_f = zeros(N,1); state.yaw_f = zeros(N,1);
  state.Qf = zeros(N,4);

  state.gyr_meas = zeros(N,3);
  state.acc_meas = zeros(N,3);

  state.cmd_angvel = zeros(N,3);
  state.cmd_after_act = zeros(N,3);

  % PID state
  state.pid.integrator = [0,0,0];
  state.pid.prev_error = [0,0,0];
  state.pid.prev_derivative = [0,0,0];

  % Wheel torque applied at each step
  state.torque_wheel_actual = zeros(N,3);   % torque actually applied by wheel
  state.torque_wheel_desired = zeros(N,3);  % PID-desired torque on body

  % Wheel speeds over time (N×3) and motor torque command history
  state.wheel_angvel = zeros(N,3);         % rad/s, wheel spin rates logged
  state.tau_motor_cmd = zeros(N,3);        % commanded motor torque (N·m)
  state.wheel_angle  = zeros(N,3);         % rad, integrated wheel rotation (for visualization)
  % initial wheel speed (0)
  state.wheel_angvel(1,:) = [0,0,0];
  state.wheel_angle(1,:) = [0,0,0];

  % Initial true state: level + initial yaw
  state.angle_true(1,:) = [0, 0, params.INITIAL_YAW];
  state.angvel_true(1,:) = [0,0,0];

  % Initial sensors from true state
  state.gyr_meas(1,:) = state.angvel_true(1,:);
  state.acc_meas(1,:) = true_accel_from_euler(state.angle_true(1,1), state.angle_true(1,2), state.angle_true(1,3), params.G);

  % Initial filter estimate from accel
  state.roll_f(1)  = atan2(state.acc_meas(1,2), state.acc_meas(1,3));
  state.pitch_f(1) = atan2(-state.acc_meas(1,1), sqrt(state.acc_meas(1,2)^2 + state.acc_meas(1,3)^2 + params.EPS));
  state.yaw_f(1)   = params.INITIAL_YAW;
  state.Qf(1,:) = eul2quat(state.roll_f(1), state.pitch_f(1), state.yaw_f(1));
  state.Qf(1,:) = normalize_quat(state.Qf(1,:));

  state.Q_true = state.Qf(1,:);  % true attitude quaternion

  % Scale integrator limits based on actuator
  state.PID_integrator_min = params.PID.integrator_min_frac * 1;
  state.PID_integrator_max = params.PID.integrator_max_frac * 1;
endfunction

% -------------------------------------------------------------------------
% Simulation loop
% -------------------------------------------------------------------------
function [state, params] = run_simulation(state, params)
  N = params.N;
  DT = params.DT;

  for k = 2:N
    % --- Step 1: Complementary filter (quaternion-only) ---
    q_prev = state.Qf(k-1,:)';
    qf = comp_step_quat(q_prev, state.gyr_meas(k,:), state.acc_meas(k,:), params.comp);
    state.Qf(k,:) = qf';

    % --- Step 2: Compute quaternion setpoint from setpoint Euler angles ---
    q_setpoint = eul2quat(state.setpoint(k,1), state.setpoint(k,2), state.setpoint(k,3))';

    % --- Step 3: PID in quaternion error space ---
    [tau_desired_body, state.pid] = pid_quat(qf, q_setpoint, state.pid, params, params.DT);

    % --- Step 4: Torque allocation to wheels ---
    Nmat = params.WHEEL_AXIS'; % 3x3 where column i = wheel axis vector
    reg = 1e-6; % regularization to avoid singularities
    tau_motor = -pinv(Nmat' * Nmat + reg*eye(3)) * Nmat' * tau_desired_body;

    % Clamp motor torques to limits
    for i = 1:3
      tau_motor(i) = clamp(tau_motor(i), -params.FLYWHEEL.torque_max(i), params.FLYWHEEL.torque_max(i));
    end
    state.tau_motor_cmd(k,:) = tau_motor';

    % --- Step 5: Integrate wheel dynamics ---
    Jw_vec = params.FLYWHEEL.Jw(:);
    wheel_w_prev = state.wheel_angvel(k-1,:)';
    wheel_w_dot = zeros(3,1);
    for i = 1:3
      b = 1e-3; % small viscous friction
      wheel_w_dot(i) = (tau_motor(i) - b*wheel_w_prev(i)) / Jw_vec(i);
      state.wheel_angvel(k,i) = wheel_w_prev(i) + wheel_w_dot(i)*DT;
      % enforce wheel speed limits
      state.wheel_angvel(k,i) = clamp(state.wheel_angvel(k,i), -params.FLYWHEEL.wheel_angvel_max(i), params.FLYWHEEL.wheel_angvel_max(i));
      % integrate wheel angle for visualization
      state.wheel_angle(k,i) = state.wheel_angle(k-1,i) + state.wheel_angvel(k,i)*DT;
    endfor

    % --- Step 6: Compute reaction torque from wheels ---
    tau_from_wheels = zeros(3,1);
    for i = 1:3
      ni = params.WHEEL_AXIS(i,:)';
      tau_from_wheels = tau_from_wheels - tau_motor(i) * ni;
      state.torque_wheel_desired(k,i) = (-tau_desired_body' * ni); % diagnostic
      state.torque_wheel_actual(k,i) = tau_motor(i);
    endfor

    % --- Step 7: Integrate body rotational dynamics ---
    omega_prev = state.angvel_true(k-1,:)';
    I = params.I_body;
    gyro_term = cross(omega_prev, I*omega_prev);
    omega_dot = params.I_body_inv * (tau_from_wheels - gyro_term);
    omega_new = omega_prev + omega_dot*DT;
    state.angvel_true(k,:) = omega_new';

    % --- Step 8: Update true quaternion using body rates ---
    wx = omega_new(1); wy = omega_new(2); wz = omega_new(3);
    Omega = 0.5 * [  0, -wx, -wy, -wz;
                     wx,  0,  wz, -wy;
                     wy, -wz,  0,  wx;
                     wz,  wy, -wx,  0 ];
    q_dot = Omega * q_prev;
    q_new = q_prev + q_dot*DT;
    q_new = q_new / norm(q_new + 1e-12);
    state.Qf(k,:) = q_new';

    % --- Step 9: Update Euler angles for plotting only ---
    eul = quat2eul(q_new'); % convert for plots
    state.angle_true(k,:) = eul;
    state.roll_f(k) = eul(1); state.pitch_f(k) = eul(2); state.yaw_f(k) = eul(3);
  endfor
endfunction

% -------------------------------------------------------------------------
% Plotting helper
% -------------------------------------------------------------------------
function plot_results(state, params)
  IDX_X = params.IDX_X; IDX_Y = params.IDX_Y; IDX_Z = params.IDX_Z;
  t = state.t;
  deg = @(x) (180/pi) * x;

  setpoint_deg = deg(state.setpoint);
  angle_true_deg = deg(state.angle_true);
  angle_filt_deg = deg([state.roll_f, state.pitch_f, state.yaw_f]);

  % Angles: 3 subplots
  figure('Name','Angles: Setpoint vs True vs Filtered','NumberTitle','off');
  subplot(3,1,1);
  plot(t, setpoint_deg(:,IDX_X), '--', 'LineWidth', 1.2); hold on;
  plot(t, angle_true_deg(:,IDX_X), '-', 'LineWidth', 1.0);
  plot(t, angle_filt_deg(:,IDX_X), ':', 'LineWidth', 1.2);
  grid on; ylabel('Roll (deg)'); legend('setpoint','true','filtered','Location','NorthEast'); title('Roll');

  subplot(3,1,2);
  plot(t, setpoint_deg(:,IDX_Y), '--', 'LineWidth', 1.2); hold on;
  plot(t, angle_true_deg(:,IDX_Y), '-', 'LineWidth', 1.0);
  plot(t, angle_filt_deg(:,IDX_Y), ':', 'LineWidth', 1.2);
  grid on; ylabel('Pitch (deg)'); legend('setpoint','true','filtered','Location','NorthEast'); title('Pitch');

  subplot(3,1,3);
  plot(t, setpoint_deg(:,IDX_Z), '--', 'LineWidth', 1.2); hold on;
  plot(t, angle_true_deg(:,IDX_Z), '-', 'LineWidth', 1.0);
  plot(t, angle_filt_deg(:,IDX_Z), ':', 'LineWidth', 1.2);
  grid on; ylabel('Yaw (deg)'); xlabel('Time (s)'); legend('setpoint','true','filtered','Location','NorthEast'); title('Yaw');

  % Angular velocity commands
  figure('Name','Angular velocity commands','NumberTitle','off');
  cmd_before_deg = deg(state.cmd_angvel);
  angvel_true_deg = deg(state.angvel_true);

  subplot(3,1,1);
  plot(t, cmd_before_deg(:,IDX_X), '--', 'LineWidth', 1.0); hold on;
  plot(t, angvel_true_deg(:,IDX_X), '-', 'LineWidth', 1.0);
  grid on; ylabel('Roll \omega (deg/s)');
  legend('cmd raw','true \omega','Location','NorthEast'); title('Flywheel Angular Velocity: Roll');

  subplot(3,1,2);
  plot(t, cmd_before_deg(:,IDX_Y), '--', 'LineWidth', 1.0); hold on;
  plot(t, angvel_true_deg(:,IDX_Y), '-', 'LineWidth', 1.0);
  grid on; ylabel('Pitch \omega (deg/s)');
  legend('cmd raw','true \omega','Location','NorthEast'); title('Flywheel Angular Velocity: Pitch');

  subplot(3,1,3);
  plot(t, cmd_before_deg(:,IDX_Z), '--', 'LineWidth', 1.0); hold on;
  plot(t, angvel_true_deg(:,IDX_Z), '-', 'LineWidth', 1.0);
  grid on; ylabel('Yaw \omega (deg/s)');
  xlabel('Time (s)');
  legend('cmd raw','true \omega','Location','NorthEast'); title('Flywheel Angular Velocity: Yaw');

  figure('Name','Flywheel Torque','NumberTitle','off');
  torque_desired_deg = state.torque_wheel_desired;
  torque_actual_deg = state.torque_wheel_actual;

  subplot(3,1,1);
  plot(state.t, torque_desired_deg(:,params.IDX_X), '--', 'LineWidth', 1.2); hold on;
  plot(state.t, torque_actual_deg(:,params.IDX_X), '-', 'LineWidth', 1.2);
  grid on; ylabel('Roll Torque (N/m)'); title('Flywheel Torque: Roll');
  legend('desired','actual','Location','NorthEast');

  subplot(3,1,2);
  plot(state.t, torque_desired_deg(:,params.IDX_Y), '--', 'LineWidth', 1.2); hold on;
  plot(state.t, torque_actual_deg(:,params.IDX_Y), '-', 'LineWidth', 1.2);
  grid on; ylabel('Pitch Torque (N/m)'); title('Flywheel Torque: Pitch');
  legend('desired','actual','Location','NorthEast');

  subplot(3,1,3);
  plot(state.t, torque_desired_deg(:,params.IDX_Z), '--', 'LineWidth', 1.2); hold on;
  plot(state.t, torque_actual_deg(:,params.IDX_Z), '-', 'LineWidth', 1.2);
  grid on; ylabel('Yaw Torque (N/m)'); xlabel('Time (s)'); title('Flywheel Torque: Yaw');
  legend('desired','actual','Location','NorthEast');

  drawnow();
endfunction

% -------------------------------------------------------------------------
% Reusable utility functions
% -------------------------------------------------------------------------

function acc_body = true_accel_from_euler(roll, pitch, yaw, G)
  % Compute accelerometer reading in body frame due to gravity only.
  ax = -G * sin(pitch);
  ay =  G * sin(roll) * cos(pitch);
  az =  G * cos(roll) * cos(pitch);
  acc_body = [ax, ay, az];
endfunction

function qf = comp_step_quat(q_prev, gyr, acc, params)
  % Complementary filter fully in quaternion
  DT = params.DT; ALPHA = params.ALPHA; EPS = params.EPS;

  % --- Propagate quaternion using gyro ---
  wx = gyr(params.IDX_X); wy = gyr(params.IDX_Y); wz = gyr(params.IDX_Z);
  Omega = 0.5 * [  0, -wx, -wy, -wz;
                   wx,   0,  wz, -wy;
                   wy, -wz,   0,  wx;
                   wz,  wy, -wx,   0 ];
  q_dot = Omega * q_prev;
  q_gyro = q_prev + q_dot * DT;
  q_gyro = q_gyro / (norm(q_gyro) + 1e-12);

  % --- Accel quaternion (roll/pitch only) ---
  ax = acc(params.IDX_X); ay = acc(params.IDX_Y); az = acc(params.IDX_Z);
  roll_acc  = atan2(ay, az);
  pitch_acc = atan2(-ax, sqrt(ay*ay + az*az + EPS));
  q_acc = eul2quat(roll_acc, pitch_acc, 0)';
  q_acc = q_acc / (norm(q_acc) + 1e-12);

  % --- Complementary filter: SLERP between gyro and accel ---
  qf = slerp(q_gyro, q_acc, 1 - ALPHA);
  qf = qf / (norm(qf) + 1e-12);  % normalize
endfunction

function q_interp = slerp(q1, q2, t)
  % Spherical linear interpolation between quaternions q1->q2
  % q1, q2 are 4x1 column vectors
  dotp = dot(q1, q2);
  if dotp < 0
      q2 = -q2; dotp = -dotp;  % take shortest path
  end
  dotp = min(max(dotp, -1), 1);  % clamp for acos
  theta = acos(dotp);
  if theta < 1e-6
      q_interp = q1;  % nearly identical
      return;
  end
  q_interp = (sin((1-t)*theta)/sin(theta))*q1 + (sin(t*theta)/sin(theta))*q2;
endfunction

function plot_3d_floatsat(state, params)
    figure('Name','3D FloatSat Visualization','NumberTitle','off');
    axis equal
    grid on
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3)
    hold on

    % Fixed axes limits
    axis_limit = 0.2;
    axis([-axis_limit axis_limit -axis_limit axis_limit -axis_limit axis_limit]);
    axis manual;

    % --- Sphere (body frame) ---
    sphere_radius = 0.0633;   % requested radius
    sphere_res = 32;

    [Xs, Ys, Zs] = sphere(sphere_res);
    Xs *= sphere_radius;
    Ys *= sphere_radius;
    Zs *= sphere_radius;

    % Convert grid → vertices Nx3
    verts_sphere = [Xs(:), Ys(:), Zs(:)];

    % Build triangle faces (each quad -> 2 tris)
    faces = [];
    n = sphere_res + 1;
    for i = 1:(n-1)
      for j = 1:(n-1)
        v1 = (i-1)*n + j;
        v2 = v1 + 1;
        v3 = v1 + n;
        v4 = v3 + 1;
        faces = [faces;
                 v1, v2, v3;
                 v2, v4, v3];
      endfor
    endfor

    % Create patch for body
    hBody = patch('Vertices', verts_sphere, 'Faces', faces, ...
                  'FaceColor', 'cyan', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

    % -------------------------
    % Arrow mesh creation (shaft + cone)
    % -------------------------
    % Arrow points along +Z in local (body) coords, base at origin.
    arrow_length = sphere_radius * 1.6;   % visual length
    shaft_frac = 0.75;                    % fraction of length used by shaft
    shaft_len = arrow_length * shaft_frac;
    cone_len  = arrow_length * (1 - shaft_frac);
    shaft_rad = sphere_radius * 0.12;
    cone_rad  = shaft_rad * 2.0;
    nseg = 24;

    [arrow_verts, arrow_faces] = make_arrow_mesh(shaft_rad, shaft_len, cone_rad, cone_len, nseg);
    % arrow_verts is Nv x 3, faces is M x 3 (triangles)

    % Create two patch handles (current = red, setpoint = green)
    hCurArrow = patch('Vertices', arrow_verts, 'Faces', arrow_faces, ...
                      'FaceColor', [1,0,0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    hSPArrow  = patch('Vertices', arrow_verts, 'Faces', arrow_faces, ...
                      'FaceColor', [0,0.7,0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);

    % ---- PERPENDICULAR ARROWS (to show yaw) ----
    % rotate +Z arrow to point along +X in body frame (so yaw rotates it in the XY plane)
    R_zto_x = axis_angle_rot([0;1;0], -pi/2);  % rotate -90deg about Y: +Z -> +X

    perp_scale = 1.0;
    perp_verts = (R_zto_x * (arrow_verts' * perp_scale))'; % scaled & rotated arrow verts

    % Perp arrow patches
    hCurPerp = patch('Vertices', perp_verts, 'Faces', arrow_faces, ...
                     'FaceColor', [1,0.1,0.1], 'EdgeColor', 'none', 'FaceAlpha', 1.0); % orange-ish
    hSPPerp  = patch('Vertices', perp_verts, 'Faces', arrow_faces, ...
                     'FaceColor', [0.1,1,0.1], 'EdgeColor', 'none', 'FaceAlpha', 1.0); % light green

    % small center marker
    plot3(0,0,0, 'k.', 'MarkerSize', 10);

    % Prepare flywheel meshes in local wheel coords (same as your code)
    n_wheels = size(params.WHEEL_POS,1);
    wheel_mesh = cell(n_wheels,1);
    wheel_faces = cell(n_wheels,1);
    wheel_Ralign = cell(n_wheels,1);
    wheel_pos = cell(n_wheels,1);
    wheel_handle = zeros(1,n_wheels);

    % choose geometry for wheels
    wheel_radius = 0.03;
    wheel_length = 0.025;

    for i = 1:n_wheels
      [verts_cyl, faces_cyl] = make_cylinder_mesh(wheel_radius, wheel_length, 24);
      wheel_mesh{i} = verts_cyl; wheel_faces{i} = faces_cyl;
      ni = params.WHEEL_AXIS(i,:)';
      % precompute alignment rotation from +Z to ni (in body frame)
      if norm(ni - [0;0;1]) < 1e-6
        Ralign = eye(3);
      else
        rot_axis = cross([0;0;1], ni);
        s = norm(rot_axis);
        c = dot([0;0;1], ni);
        if s < 1e-9
          Ralign = eye(3);
        else
          Ralign = axis_angle_rot(rot_axis / s, atan2(s, c));
        end
      end
      wheel_Ralign{i} = Ralign;
      wheel_pos{i} = params.WHEEL_POS(i,:)'; % in body frame
      % create initial patch (will replace vertices every frame)
      wheel_handle(i) = patch('Vertices',verts_cyl,'Faces',faces_cyl,'FaceColor',[0.6,0.6,0.6],'FaceAlpha',1.0);
    endfor

    % axis we visualize (body +Z)
    body_axis = [0;0;1];

    % -------------------------
    % Main loop: update body, wheels and arrows
    % -------------------------
    N = length(state.t);
    for k = 1:N
        q = state.Qf(k,:); % quaternion [w x y z]
        R_body_to_world = quat2rotm(q);   % 3x3 rotation matrix

        % Body (sphere) update: transform all vertices from body frame to world
        verts_body = verts_sphere';            % 3 x Nv
        verts_world = (R_body_to_world * verts_body)'; % Nv x 3
        set(hBody, 'Vertices', verts_world);

        % ---- update current arrow (rotate arrow mesh by body rotation) ----
        verts_arrow_body = (R_body_to_world * arrow_verts')'; % Nv x 3
        set(hCurArrow, 'Vertices', verts_arrow_body);
        verts_perp_body = (R_body_to_world * perp_verts')'; % transform to world
        set(hCurPerp, 'Vertices', verts_perp_body);

        % ---- update setpoint arrow (compute setpoint rotation, rotate arrow) ----
        q_sp = eul2quat(state.setpoint(k,1), state.setpoint(k,2), state.setpoint(k,3))';
        R_sp = quat2rotm(q_sp);
        verts_arrow_sp = (R_sp * arrow_verts')';
        set(hSPArrow, 'Vertices', verts_arrow_sp);
        verts_perp_sp = (R_sp * perp_verts')'; % transform to world
        set(hSPPerp, 'Vertices', verts_perp_sp);

        % Wheels: transform and spin (unchanged)
        for i = 1:n_wheels
            verts_cyl = wheel_mesh{i}'; % 3 x Nv (in local cyl coords)
            angle = state.wheel_angle(k,i);
            ni = params.WHEEL_AXIS(i,:)';
            Rspin = axis_angle_rot(ni, angle); % rotate about ni in body frame
            verts_body_cyl = (Rspin * (wheel_Ralign{i} * verts_cyl));
            pos_body = wheel_pos{i};
            verts_body_cyl = verts_body_cyl + repmat(pos_body, 1, size(verts_body_cyl,2));
            verts_world = (R_body_to_world * verts_body_cyl)';
            set(wheel_handle(i), 'Vertices', verts_world);
        endfor

        drawnow;
        pause(params.DT);
    endfor
endfunction

function R = axis_angle_rot(axis, theta)
  % Rodrigues rotation matrix: rotate around (3x1) axis (unit) by theta radians.
  if norm(axis) < 1e-12
    R = eye(3);
    return;
  end
  u = axis / norm(axis);
  ux = [   0, -u(3),  u(2);
         u(3),    0, -u(1);
        -u(2), u(1),    0 ];
  R = eye(3) * cos(theta) + (1 - cos(theta)) * (u * u') + ux * sin(theta);
endfunction

function [verts, faces] = make_cylinder_mesh(radius, length, nseg)
  if nargin < 3, nseg = 24; end

  % Get cylinder grid (cylinder returns nseg+1 columns: last == first)
  [X, Y] = cylinder(radius, nseg);
  % remove duplicated seam column so we have exactly nseg samples around
  X = X(:,1:end-1);
  Y = Y(:,1:end-1);

  nz = size(X,1);           % number of rings along Z (usually 2)
  nv_per_ring = nseg;       % now exactly nseg vertices per ring

  % Z values for each ring (nz x nv_per_ring)
  Z = repmat(linspace(-length/2, length/2, nz)', 1, nv_per_ring);

  % build vertex list (flatten column-major: first column then second)
  verts = [X(:), Y(:), Z(:)];

  % correct index function for column-major flattening:
  % for (r,c): index = (c-1)*nz + r
  idx = @(r,c) (c-1)*nz + r;

  % preallocate face array
  n_side_tris = (nz-1) * nv_per_ring * 2;
  n_cap_tris  = nv_per_ring * 2;
  total_tris  = n_side_tris + n_cap_tris;
  faces = zeros(total_tris, 3);
  p = 1;

  % --- Side faces (two triangles per quad) ---
  for r = 1:(nz-1)
    for c = 1:nv_per_ring
      c2 = mod(c, nv_per_ring) + 1; % next column (wrap)
      a = idx(r, c);
      b = idx(r, c2);
      d = idx(r+1, c);
      cidx = idx(r+1, c2);
      % triangle 1
      faces(p, :) = [a, b, d]; p = p + 1;
      % triangle 2
      faces(p, :) = [b, cidx, d]; p = p + 1;
    endfor
  endfor

  % --- Caps (triangulate as fan) ---
  top_center_idx = size(verts,1) + 1;
  bottom_center_idx = top_center_idx + 1;
  verts = [verts; 0,0, length/2; 0,0, -length/2];

  % top cap (ring r = nz), winding so normal -> +Z
  for c = 1:nv_per_ring
    c2 = mod(c, nv_per_ring) + 1;
    a = idx(nz, c);
    b = idx(nz, c2);
    faces(p, :) = [a, b, top_center_idx]; p = p + 1;
  endfor

  % bottom cap (ring r = 1), reverse order so normal -> -Z
  for c = 1:nv_per_ring
    c2 = mod(c, nv_per_ring) + 1;
    a = idx(1, c);
    b = idx(1, c2);
    faces(p, :) = [b, a, bottom_center_idx]; p = p + 1;
  endfor
endfunction

function [verts, faces] = make_arrow_mesh(shaft_rad, shaft_len, cone_rad, cone_len, nseg)
  if nargin < 5, nseg = 24; end
  % Rings:
  % bottom ring at z=0 (base)
  z0 = 0;
  z1 = shaft_len;        % ring where cone starts
  z2 = shaft_len + cone_len; % apex z

  theta = linspace(0, 2*pi, nseg+1);
  theta(end) = []; % remove duplicate
  % bottom ring
  xb = shaft_rad * cos(theta)'; yb = shaft_rad * sin(theta)'; zb = z0 * ones(nseg,1);
  % top ring (shaft->cone start) use shaft radius
  xt = shaft_rad * cos(theta)'; yt = shaft_rad * sin(theta)'; zt = z1 * ones(nseg,1);
  % cone ring (base of cone) uses cone_rad at z1
  xc = cone_rad  * cos(theta)'; yc = cone_rad  * sin(theta)'; zc = z1 * ones(nseg,1);
  apex = [0,0,z2];

  % assemble verts (Nv x 3)
  verts = [xb, yb, zb;
           xt, yt, zt;
           xc, yc, zc;
           apex];

  % indices
  idx_b = 1:nseg;
  idx_t = nseg + (1:nseg);
  idx_c = 2*nseg + (1:nseg);
  idx_ap = 3*nseg + 1;

  faces = [];
  % shaft side: each quad -> 2 triangles
  for c = 1:nseg
    a = idx_b(c);
    b = idx_b(mod(c, nseg) + 1);
    d = idx_t(c);
    cidx = idx_t(mod(c, nseg) + 1);
    % two triangles: a-b-d and b-cidx-d
    faces = [faces;
             a, b, d;
             b, cidx, d];
  endfor

  % seam triangles already handled by modulo indexing

  % cone side: triangles from cone ring to apex
  for c = 1:nseg
    a = idx_c(c);
    b = idx_c(mod(c, nseg) + 1);
    faces = [faces; a, b, idx_ap];
  endfor

  % optional: cap bottom (triangulate to origin) so arrow looks solid if seen from below
  bottom_center_idx = size(verts,1) + 1;
  verts = [verts; 0, 0, z0];
  for c = 1:nseg
    a = idx_b(c);
    b = idx_b(mod(c, nseg) + 1);
    faces = [faces; a, b, bottom_center_idx];
  endfor
endfunction

function [tau_desired_body, pid_state] = pid_quat(q_current, q_setpoint, pid_state, params, DT)
  % Compute body torque using quaternion error
  % q_current, q_setpoint: 4x1 quaternions
  % pid_state: struct with .integrator, .prev_error (3x1)
  % Returns 3x1 torque vector

  % --- Quaternion error (rotation from current -> setpoint) ---
  q_err = quatmultiply(quatconjugate(q_current'), q_setpoint'); % 1x4
  if q_err(1) < 0
      q_err = -q_err;  % shortest rotation
  end
  % rotation vector approximation: small angle
  rot_err = q_err(2:4) * 2;  % 3x1 vector

  tau_desired_body = zeros(3,1);
  for axis = 1:3
      err = rot_err(axis);
      % PID
      pid_state.integrator(axis) = pid_state.integrator(axis) + err * DT * params.PID.Ki(axis);
      pid_state.integrator(axis) = clamp(pid_state.integrator(axis), ...
                                         params.PID.integrator_min_frac, ...
                                         params.PID.integrator_max_frac);
      derivative_raw = (err - pid_state.prev_error(axis)) / DT;
      tau = params.PID.tau_D;
      D_filtered = (tau * pid_state.prev_derivative(axis) + DT * derivative_raw) / (tau + DT);
      pid_state.prev_derivative(axis) = D_filtered;
      pid_state.prev_error(axis) = err;
      tau_desired_body(axis) = params.PID.Kp(axis)*err + pid_state.integrator(axis) + params.PID.Kd(axis)*D_filtered;
  end
endfunction

function q_conj = quatconjugate(q)
  % q = [w x y z] row or column
  q_conj = [q(1), -q(2), -q(3), -q(4)];
endfunction

function q_out = quatmultiply(q1, q2)
  % Hamilton product: q1 * q2
  % q1, q2 = [w x y z] row vectors
  w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
  w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);

  w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
  x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
  y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
  z = w1*z2 + x1*y2 - y1*x2 + z1*w2;

  q_out = [w x y z];
endfunction

function R = quat2rotm(q)
    % Convert quaternion [w x y z] to rotation matrix
    w = q(1); x = q(2); y = q(3); z = q(4);
    R = [1-2*(y^2+z^2), 2*(x*y - z*w), 2*(x*z + y*w);
         2*(x*y + z*w), 1-2*(x^2+z^2), 2*(y*z - x*w);
         2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x^2+y^2)];
endfunction

% Run the main entry point automatically when the file is executed
main();
