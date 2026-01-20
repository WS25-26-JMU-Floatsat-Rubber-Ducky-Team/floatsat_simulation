function q_conj = quatconjugate(q)
  % q = [w x y z] row or column
  q_conj = [q(1), -q(2), -q(3), -q(4)];
endfunction
