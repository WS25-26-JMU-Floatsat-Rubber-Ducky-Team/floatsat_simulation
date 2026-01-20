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
