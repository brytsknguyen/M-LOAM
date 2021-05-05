function [R] = rot_opt(gndtr, rot_est)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

N = size(gndtr, 3);

A = repmat(eye(3), [1 N]);
B = [];
for n=1:N
    B = [B, rot_est(:, :, n)*gndtr(:, :, n)'];
end


[U, D, V] = svd(A*B');

S = eye(3);
if det(U)*det(V) < 0
    S(3, 3) = -1;
end

R = U*S*V';

end

