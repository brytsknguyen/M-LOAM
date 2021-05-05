function Mo = vecitp(Mi, ti, to, idx)
%UNTITLED Summary of this function goes here

s  = (to - ti(idx(:, 1)))./(ti(idx(:, 2)) - ti(idx(:, 1)));

% if mean(find(s < 1)) ~= 0
%     fprintf('WARNING, intp factor > 1!\n');
% end

dM  = Mi(idx(:, 2), :) - Mi(idx(:, 1), :);
sdM = [s.*dM(:, 1), s.*dM(:, 2), s.*dM(:, 3)];

Mo = Mi(idx(:, 1), :) + sdM;

end