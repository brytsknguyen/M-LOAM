function [P_h_ate, rot_h_rmse] ...
            = check_viral_cal_err_parall(test_id, test_fullname)

% clearvars('-except', vars_to_keep{:});
close all;

% addpath('/home/tmn/MATLAB_WS');

t_shift = 0;

myorange = [0.96, 0.96, 0.00];
mycyan   = [0.00, 0.75, 1.00];

fighd = [];

%% get the exp name number
exp_name =  test_fullname.name;
exp_path = [test_fullname.folder '/' test_fullname.name '/'];


gndtr_pos_fn       = [exp_path 'leica_pose.csv'];
gndtr_dji_imu_fn   = [exp_path 'dji_sdk_imu.csv'];
gndtr_vn100_imu_fn = [exp_path 'vn100_imu.csv'];
pose_est_fn        = [exp_path 'predict_odom.csv'];
trans_B2prism_fn   = [exp_path '../trans_B2prism.csv'];
tf_B2Bhloam_fn     = [exp_path '../tf_B2Bhloam.csv'];
rot_B2vn100_fn     = [exp_path '../rot_B2vn100.csv'];
rot_B2djiimu_fn    = [exp_path '../rot_B2djiimu.csv'];


%% Read the data from log and start processing


%% Read the gndtr data from logs

% Position groundtr
gndtr_pos_data = csvread(gndtr_pos_fn,  1, 0);

% Orientation groundtr
% Check if file size is 0
dji_file   = dir(gndtr_dji_imu_fn);
vn100_file = dir(gndtr_vn100_imu_fn);
imu_topic = '';
% Orientation is in z upward frame, convert it to body frame
rot_B_Beimu = eye(3);

dji_present   = (dji_file.bytes ~= 0);
vn100_present = (vn100_file.bytes ~= 0);

if vn100_present
    gndtr_vn100_data = csvread([exp_path 'vn100_imu.csv'], 1, 0);
end

if dji_present
    gndtr_dji_data = csvread(gndtr_dji_imu_fn, 1, 0);
    rot_B_Beimu    = csvread(rot_B2djiimu_fn, 0, 0);
    imu_topic = '/dji_sdk/imu';
elseif ~dji_present && vn100_present
    gndtr_dji_data = csvread([exp_path 'vn100_imu.csv'], 1, 0);
    rot_B_Beimu    = csvread(rot_B2vn100_fn,  0, 0);
    imu_topic = '/imu/imu';
end

t0_ns = gndtr_pos_data(1, 1);

% pos groundtruthdata
t_pos = (gndtr_pos_data(:, 1) - t0_ns)/1e9 + t_shift;
P     = gndtr_pos_data(:, 4:6);

% ori groundtruthdata
t_ori = (gndtr_dji_data(:, 1)- t0_ns)/1e9;
Q     =  quatnormalize(gndtr_dji_data(:, [7, 4:6]));
Q0    =  quatnormalize(Q(1, :));

% Delete the duplicate in position groundtruth data
[~, Px_unq_idx] = unique(P(:, 1));
[~, Py_unq_idx] = unique(P(:, 2));
[~, Pz_unq_idx] = unique(P(:, 3));

P_unq_idx = union(union(Px_unq_idx, Py_unq_idx), Pz_unq_idx);
P = P(P_unq_idx, :);
t_pos = t_pos(P_unq_idx, :);

% Delete the duplicate in orientation groundtruth data
[~, Qx_unq_idx] = unique(Q(:, 1));
[~, Qy_unq_idx] = unique(Q(:, 2));
[~, Qz_unq_idx] = unique(Q(:, 3));
[~, Qw_unq_idx] = unique(Q(:, 4));

Q_unq_idx = union(union(union(Qx_unq_idx, Qy_unq_idx), Qz_unq_idx),...
                        Qw_unq_idx);
Q     = Q(Q_unq_idx, :);
t_ori = t_ori(Q_unq_idx, :);



%% Calculate transform from lidars to the prism


% Position of prism in the body frame
trans_B2prism = (csvread(trans_B2prism_fn, 0, 0))';


% Horizontal lidar transform
tf_B2Bhloam    = csvread(tf_B2Bhloam_fn, 0, 0);
rot_B2Bhloam   = tf_B2Bhloam(1:3, 1:3);
trans_B2Bhloam = tf_B2Bhloam(1:3, 4);
% Displacement from the horz lidar to the prism
trans_Bhloam2prism = rot_B2Bhloam'*(trans_B2prism - trans_B2Bhloam);
rot_Bhloam2Beimu   = rot_B2Bhloam'*rot_B_Beimu;



%% Read the viralslam estimate data from logs
% SLAM estimate
viralslam_data = csvread(pose_est_fn, 1, 0);
t_h = (viralslam_data(:, 1) - t0_ns)/1e9;
P_h = viralslam_data(:, 4:6);
Q_h = quatnormalize(viralslam_data(:, [10, 7:9]));
V_h = viralslam_data(:, 11:13);

% Compensate the position estimate with the prism displacement
P_h = P_h + quatconv(Q_h, trans_Bhloam2prism');
% Compensate the orientation estimate with the prism displacement
Q_h = quatmultiply(Q_h, rotm2quat(rot_Bhloam2Beimu));



%% Resample the ground truth data by each estimate data
% Note affix rs[x] is for resampled by [x]


%% Resample gndtruth by viralslam time

% Find the interpolated time stamps
[rsh_pos_itp_idx(:, 1), rsh_pos_itp_idx(:, 2)] = combteeth(t_h, t_pos);
[rsh_ori_itp_idx(:, 1), rsh_ori_itp_idx(:, 2)] = combteeth(t_h, t_ori);

% Remove the un-associatable samples
rsh_nan_idx = find(isnan(rsh_pos_itp_idx(:, 1))...
    | isnan(rsh_pos_itp_idx(:, 2))...
    | isnan(rsh_ori_itp_idx(:, 1))...
    | isnan(rsh_ori_itp_idx(:, 2))...
    );

t_h_full = t_h;
P_h_full = P_h;
Q_h_full = Q_h;
V_h_full = V_h;

rsh_pos_itp_idx(rsh_nan_idx, :) = [];
rsh_ori_itp_idx(rsh_nan_idx, :) = [];
t_h(rsh_nan_idx, :)     = [];
P_h(rsh_nan_idx, :)     = [];
Q_h(rsh_nan_idx, :)     = [];
V_h(rsh_nan_idx, :)     = [];

% interpolate the pos gndtr state
P_rsh = vecitp(P,  t_pos, t_h, rsh_pos_itp_idx);

[rot_align_h, trans_align_h ] = traj_align(P_rsh, P_h);

% Align the position estimate
P_h      = (rot_align_h*P_h'      + trans_align_h)';
P_h_full = (rot_align_h*P_h_full' + trans_align_h)';

% Align the velocity estimate
V_h      = (rot_align_h*V_h')';
V_h_full = (rot_align_h*V_h_full')';

% interpolate the ori gndtr state
Q_rsh = quatitp(Q, t_ori, t_h, rsh_ori_itp_idx);


% Find the optimized rotation between the groundtruth and the estimate
rot_rsh = quat2rotm(Q_rsh);
rot_h   = quat2rotm(Q_h);

rot_rsh2h_opt = (rot_opt(rot_rsh, rot_h))';

% Align the ori estimate
Q_rsh = quatmultiply(rotm2quat(rot_rsh2h_opt), Q_rsh);



%% Log down the transforms in yaml files for the vizualization
% Export the leica transform to a yaml file
fileID = fopen([exp_name '/leica_tf.yaml'], 'w');
fprintf(fileID, ['%%YAML:1.0\n'...
                 'T_W_Wleica: !!opencv-matrix\n'...
                 '  rows: 4\n'...
                 '  cols: 4\n'...
                 '  dt: d\n']);
R_W2L   =  rot_align_h';
t_W2L   = -rot_align_h'*trans_align_h;
T_W2L   = [R_W2L, t_W2L; 0 0 0 1];
T_W2L_str = sprintf(['  data: [ %0.9f, %0.9f, %0.9f, %0.9f,\n'...
                     '          %0.9f, %0.9f, %0.9f, %0.9f,\n'...
                     '          %0.9f, %0.9f, %0.9f, %0.9f,\n'...
                     '          %0.9f, %0.9f, %0.9f, %0.9f ]\n\n'],...
                     T_W2L(1, 1), T_W2L(1, 2), T_W2L(1, 3), T_W2L(1, 4),...
                     T_W2L(2, 1), T_W2L(2, 2), T_W2L(2, 3), T_W2L(2, 4),...
                     T_W2L(3, 1), T_W2L(3, 2), T_W2L(3, 3), T_W2L(3, 4),...
                     T_W2L(4, 1), T_W2L(4, 2), T_W2L(4, 3), T_W2L(4, 4));
fprintf(fileID, T_W2L_str);

fprintf(fileID, ['T_B_Bleica: !!opencv-matrix\n'...
                 '  rows: 4\n'...
                 '  cols: 4\n'...
                 '  dt: d\n']);
             
T_B2Bleica_str = sprintf(['  data: [ %0.9f, %0.9f, %0.9f, %0.9f,\n'...
                          '          %0.9f, %0.9f, %0.9f, %0.9f,\n'...
                          '          %0.9f, %0.9f, %0.9f, %0.9f,\n'...
                          '          %0.9f, %0.9f, %0.9f, %0.9f ]'],...
                          1, 0, 0, trans_B2prism(1),...
                          0, 1, 0, trans_B2prism(2),...
                          0, 0, 1, trans_B2prism(3),...
                          0, 0, 0, 1.0);
                      
fprintf(fileID, T_B2Bleica_str);

fclose(fileID);



% Export the external IMU rotations to a yaml file

% Inertial frame rotation
R_Wh_Weimu = quat2rotm(Q_h(1, :))*rot_B_Beimu*quat2rotm(Q0)';

fileID = fopen([exp_name '/ext_imu_rot.yaml'], 'w');
fprintf(fileID, ['%%YAML:1.0\n'...
                 'imu_topic: ' imu_topic '\n'...
                 'R_W_Weimu: !!opencv-matrix\n'...
                 '  rows: 3\n'...
                 '  cols: 3\n'...
                 '  dt: d\n']);
R_Wviralba_Weimu_str...
    = sprintf(['  data: [ %0.9f, %0.9f, %0.9f,\n'...
               '          %0.9f, %0.9f, %0.9f,\n'...
               '          %0.9f, %0.9f, %0.9f ]'],...
               R_Wh_Weimu(1, 1), R_Wh_Weimu(1, 2), R_Wh_Weimu(1, 3),...
               R_Wh_Weimu(2, 1), R_Wh_Weimu(2, 2), R_Wh_Weimu(2, 3),...
               R_Wh_Weimu(3, 1), R_Wh_Weimu(3, 2), R_Wh_Weimu(3, 3));
fprintf(fileID, R_Wviralba_Weimu_str);

% Body rotation
fprintf(fileID, ['\n'...
                 'R_B_Beimu: !!opencv-matrix\n'...
                 '  rows: 3\n'...
                 '  cols: 3\n'...
                 '  dt: d\n']);
rot_B_Beimu_str...
    = sprintf(['  data: [ %0.9f, %0.9f, %0.9f,\n'...
               '          %0.9f, %0.9f, %0.9f,\n'...
               '          %0.9f, %0.9f, %0.9f ]'],...
               rot_B_Beimu(1, 1), rot_B_Beimu(1, 2), rot_B_Beimu(1, 3),...
               rot_B_Beimu(2, 1), rot_B_Beimu(2, 2), rot_B_Beimu(2, 3),...
               rot_B_Beimu(3, 1), rot_B_Beimu(3, 2), rot_B_Beimu(3, 3));
fprintf(fileID, rot_B_Beimu_str);
fclose(fileID);



%% Calculate the position and rotation errors


%% Calculate the position and rotation errors of viralslam estimate
P_h_prism   = trans_B2prism;
P_h_err     = P_rsh - P_h;
P_h_rmse    = rms(P_h_err);
P_h_ate     = norm(P_h_rmse);
P_h_err_nrm = sqrt(dot(P_h_err, P_h_err, 2));

Q_h_err       = quatmultiply(quatinv(Q_h), Q_rsh);
YPR_h_err	  = wrapToPi(quat2eul(Q_h_err));
rot_h_ang_err = quat2axang(Q_h_err);
% Wrap this error to -pi to pi;
rot_h_ang_err(:, end) = wrapToPi(rot_h_ang_err(:, end));
% Find the outliers
olrh_idx = find(isoutlier(rot_h_ang_err(:, end), 'mean'));
% Extract the inlier errors
rot_h_ang_err_nolr = rot_h_ang_err(:, end);
t_h_nolr           = t_h;
rot_h_ang_err_nolr(olrh_idx, :) = [];
t_h_nolr(olrh_idx)              = [];
% Calculate the error
rot_h_rmse         = rms(rot_h_ang_err_nolr(:, end))/pi*180;
% rot_h_ang_err_norm = abs(rot_h_ang_err(:, end));



%% Save the important variables for later analyses
save([exp_path exp_name '_poses.mat'],...
     't_h', 'P_rsh', 'Q_rsh', 'P_h', 'Q_h', 'rot_align_h', 'trans_align_h');

% save the errors calculated.
save([exp_path exp_name '_rmse.mat'], 'P_h_ate', 'rot_h_rmse');


%% Print the result
fprintf(['test: %2d. %s. Err: P_h: %6.4f, rot_h: %7.4f.\n'],...
          test_id, exp_name(8:end), P_h_ate, rot_h_rmse);


%% Calculate the maximum time
t_max = max([t_pos; t_h]);



%% Plot the 3D trajectory
figpos = [1920 0 0 0] + [0, 480, 630, 400];
figure('position', figpos, 'color', 'w', 'paperpositionmode', 'auto');
fighd = [fighd gcf];
hold on;
plot3(P(1:2, 1), P(1:2, 2), P(1:2, 3),...
    'r', 'linewidth', 3);
plot3(P_h(1:2, 1),  P_h(1:2, 2),  P_h(1:2, 3),...
    'b', 'linewidth', 3);
plot3(P(:, 1), P(:, 2), P(:, 3),...
    '.r', 'markersize', 6);
plot3(P_h_full(:, 1),  P_h_full(:, 2),  P_h_full(:, 3),...
    '.b', 'markersize', 6);

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;
daspect([1 1 1]);
view([-21 15]);
tightfig;
set(gca, 'fontsize', 13);
% lg_hd = legend('Leica', 'LOAM (H)', 'LOAM (V)', 'viralslam');
lg_hd = legend('Leica', 'MILIOM');
set(lg_hd, 'position', [0.69925297449761,...
                        0.694882822813015,...
                        0.233789057870308,...
                        0.26499999254942]);
saveas(gcf, [exp_path exp_name '_traj.fig']);
% saveas(gcf, [exp_path exp_name '_traj.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_traj.png']);



%% Plot the time evolution of position
figpos = [1920 0 0 0] + [0, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
axgnd = plot(t_pos, P(:, 1),   'r', 'linewidth', 4);
axh   = plot(t_h,   P_h(:, 1), 'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
% xlabel('Time [s]');
ylabel('X [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
axgnd = plot(t_pos, P(:, 2),   'r', 'linewidth', 4);
axh   = plot(t_h,   P_h(:, 2), 'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
% xlabel('Time [s]');
ylabel('Y [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
axgnd = plot(t_pos, P(:, 3),   'r', 'linewidth', 3);
axh   = plot(t_h,   P_h(:, 3), 'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
xlabel('Time [s]');
ylabel('Z [m]');
% ylim([-1.5, 1.5]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);


tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyzt.fig']);
% saveas(gcf, [exp_path exp_name '_xyzt.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyzt.png']);



%% Plot the time evolution of orientation

% Calculate the yaw pitch rol relative to the initial position
YPR       = quat2eul(quatmultiply(quatinv(Q(1, :)),       Q));
YPR_h     = quat2eul(quatmultiply(quatinv(Q_h(1, :)),     Q_h));


figpos = [1920 0 0 0] + [630, 480, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
axgnd = plot(t_ori,   YPR(:, 1)*180/pi,       'r', 'linewidth', 4);
axh   = plot(t_h,     YPR_h(:, 1)*180/pi,     'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
% xlabel('Time [s]');
ylabel('Yaw [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
axgnd = plot(t_ori,   YPR(:, 2)*180/pi,       'r', 'linewidth', 4);
axh   = plot(t_h,     YPR_h(:, 2)*180/pi,     'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
% xlabel('Time [s]');
ylabel('Pitch [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
axgnd = plot(t_ori,   YPR(:, 3)*180/pi,       'r', 'linewidth', 4);
axh   = plot(t_h,     YPR_h(:, 3)*180/pi,     'b', 'linewidth', 2);
uistack(axgnd, 'top');
uistack(axh, 'top');
xlabel('Time [s]');
ylabel('Roll [deg]');
% ylim([-1.5, 1.5]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Groundtruth', 'MILIOM');
set(lg_hd, 'position', [0.740750745700882,...
                        0.249333251622547,...
                        0.214062495995313,...
                        0.239999993294478]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_yprt.fig']);
% saveas(gcf, [exp_path exp_name '_yprt.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_yprt.png']);



%% Plot the time evolution of velocity estimate
figpos = [1920 0 0 0] + [630, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

hold on;
plot(t_h, V_h(:, 1), 'r', 'linewidth', 2);
plot(t_h, V_h(:, 2), 'g', 'linewidth', 2);
plot(t_h, V_h(:, 3), 'b', 'linewidth', 2);
plot(t_h, sqrt(dot(V_h, V_h, 2)), 'g', 'linewidth', 2);
% xlabel('Time [s]');
ylabel('Vel. Est. [m/s]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Vx', 'Vy', 'Vz', 'norm (speed)');

set(lg_hd, 'orientation', 'horizontal',...
    'position', [0.2317 0.8767 0.6000 0.1000]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_vxvyvz_t.fig']);
% saveas(gcf, [exp_path exp_name '_vxvyvz_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_vxvyvz_t.png']);



%% Plot the time evolution of position error
figpos = [1920 0 0 0] + [630*2, 480, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_h,  P_h_err(:, 1),  'b', 'linewidth', 2);
ylabel('X Err. [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_h,  P_h_err(:, 2),  'b', 'linewidth', 2);
% xlabel('Time [s]');
ylabel('Y Err [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_h,  P_h_err(:, 3),  'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Z Err [m]');
% ylim([-1.5, 1.5]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('MILIOM');
set(lg_hd, 'position', [0.138344824380857,...
                        0.353488372093023,...
                        0.22,...
                        0.15]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyz_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_xyz_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyz_err_t.png']);



%% Plot the time evolution of orientation err
figpos = [1920 0 0 0] + [630*2, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_h,  YPR_h_err(:, 1)*180/pi,  'b', 'linewidth', 2);
ylabel('Yaw Err. [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_h, YPR_h_err(:, 2)*180/pi,    'b', 'linewidth', 2);
ylabel('Pitch Err. [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_h,  YPR_h_err(:, 3)*180/pi,    'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Roll Err. [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('MILIOM');

set(lg_hd, 'position', [0.138344824380857,...
                        0.353488372093023,...
                        0.22,...
                        0.15]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_ypr_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_ypr_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_ypr_err_t.png']);



%% Plot the combined time evolution of position estimation error
figpos = [1920 0 0 0] + [930, 750, 630, 200];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

hold on;
plot(t_h, P_h_err(:, 1), 'r', 'linewidth', 2);
plot(t_h, P_h_err(:, 2), 'g', 'linewidth', 2);
plot(t_h, P_h_err(:, 3), 'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Error [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Px error', 'Py error', 'Pz error');

set(lg_hd, 'orientation', 'horizontal',...
    'position', [0.3 0.85 0.4 0.1000]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyz_h_err_t.fig']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyz_h_err_t.png']);



%% Plot the combined time evolution of orientation estimation error
figpos = [1920 0 0 0] + [300, 750, 630, 200];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

hold on;
plot(t_h, YPR_h_err(:, 1)/pi*180, 'r', 'linewidth', 2);
plot(t_h, YPR_h_err(:, 2)/pi*180, 'g', 'linewidth', 2);
plot(t_h, YPR_h_err(:, 3)/pi*180, 'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Error [deg]');
ylim([-8 8]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Yaw error', 'Pitch error', 'Roll error');

set(lg_hd, 'orientation', 'horizontal',...
           'position', [0.3 0.85 0.4 0.1000]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_ypr_h_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_ypr_h_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_ypr_h_err_t.png']);

end