function Qo = quatitp(Qi, ti, to, idx)
%UNTITLED Summary of this function goes here

s  = (to - ti(idx(:, 1)))./(ti(idx(:, 2)) - ti(idx(:, 1)));

% if mean(find(s < 1)) ~= 0
%     fprintf('WARNING, intp factor > 1!\n');
% end

% Interpolate the quaternions
Qo = quatinterp(Qi(idx(:, 1), :), Qi(idx(:, 2), :), s, 'slerp');

% Normalize the quaternion
% Qo = normalize(Qo);

end