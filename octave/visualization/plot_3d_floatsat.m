function plot_3d_floatsat(state, params)
    figure('Name','3D FloatSat Visualization','NumberTitle','off');

    % out_dir = "frames_png";
    % if ~exist(out_dir, "dir")
    %     mkdir(out_dir);
    % end
    % frame_id = 0;
    % save_stride = 10;   % save 1 frame every 10 simulation steps

    % Fixed figure size (important for GIF stability)
    set(gcf, "Position", [100 100 800 800]);

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

        % -------------------------
        % Save frame
        % -------------------------
        % if mod(k, save_stride) == 0
        %     frame_id += 1;
        %     fname = sprintf("%s/frame_%05d.png", out_dir, frame_id);
        %     print(gcf, fname, "-dpng", "-r150");
        % end

        pause(params.DT);
    endfor
endfunction
