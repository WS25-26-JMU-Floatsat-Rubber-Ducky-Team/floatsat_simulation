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
