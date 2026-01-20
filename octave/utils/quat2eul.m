function eul = quat2eul(q)
  % quat2eul  Convert quaternion to ZYX Euler angles (roll, pitch, yaw).
  %
  % Usage:
  %   eul = quat2eul(q)
  % Input:
  %   q  - quaternion [w x y z] as a 1x4 row or 4x1 column vector
  % Output:
  %   eul - [roll, pitch, yaw] in radians (ZYX convention: roll = x-rot, pitch = y-rot, yaw = z-rot)
  %
  % Notes:
  %   - Handles numerical edge cases (clamps) for asin to avoid NaNs at gimbal lock.
  %
  % Example:
  %   q = eul2quat(0.1, -0.2, 0.5);
  %   e = quat2eul(q);  % should ≈ [0.1, -0.2, 0.5]
  %

  % Normalize input and accept row or column
  q = q(:)';           % ensure row vector [w x y z]
  if numel(q) ~= 4
    error('quat2eul: input must be a 4-element quaternion [w x y z]');
  end
  q = q / (norm(q) + eps);

  w = q(1); x = q(2); y = q(3); z = q(4);

  % roll (x-axis rotation)
  sinr_cosp = 2 * (w * x + y * z);
  cosr_cosp = 1 - 2 * (x*x + y*y);
  roll = atan2(sinr_cosp, cosr_cosp);

  % pitch (y-axis rotation)
  sinp = 2 * (w * y - z * x);
  % clamp to [-1,1] to avoid NaNs from numerical drift
  if sinp >= 1
    pitch = pi/2;    % gimbal lock +90 deg
  elseif sinp <= -1
    pitch = -pi/2;   % gimbal lock -90 deg
  else
    pitch = asin(sinp);
  end

  % yaw (z-axis rotation)
  siny_cosp = 2 * (w * z + x * y);
  cosy_cosp = 1 - 2 * (y*y + z*z);
  yaw = atan2(siny_cosp, cosy_cosp);

  eul = [roll, pitch, yaw];
endfunction
