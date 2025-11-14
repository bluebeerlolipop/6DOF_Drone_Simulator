addpath('./lib');
load('stateHistory_test.mat');

initState = stateHistory_test(1,:).';

%% DEFINE
R2D = 180/pi;
D2R = pi/180;

%% Simulation time
simulationTime = 2;
dt = 0.01;
N  = floor(simulationTime/dt);
t0 = tic;
next = 0;
t  = (0:N)*dt;

%gif_filename = 'drone_animation1.gif';

%% INIT Params
drone1_body = [ 0.265,      0,     0, 1; ...
                    0, -0.265,     0, 1; ...
               -0.265,      0,     0, 1; ...
                    0,  0.265,     0, 1; ...
                    0,      0,     0, 1; ...
                    0,      0, -0.15, 1]';

%% Figure(3D)
fig1 = figure('pos', [0 200 800 800]);
ax3 = axes(fig1);
view(ax3, 3);
ax3.ZDir = 'Reverse';
ax3.YDir = 'Reverse';
axis(ax3, 'equal'); grid(ax3, 'on');
xlim(ax3, [-5 5]); ylim(ax3, [-5 5]); zlim(ax3, [-8 0]);
xlabel(ax3, 'X[m]'); ylabel(ax3, 'Y[m]'); zlabel(ax3, 'Z[m]');
hold(ax3, 'on');

drone1_state = initState;
wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
drone1_world = wHb * drone1_body;
drone1_atti  = drone1_world(1:3, :);

fig1_ARM13   = plot3(ax3, drone1_atti(1,[1 3]), drone1_atti(2,[1 3]), drone1_atti(3,[1 3]), '-ro', 'MarkerSize', 5);
fig1_ARM24   = plot3(ax3, drone1_atti(1,[2 4]), drone1_atti(2,[2 4]), drone1_atti(3,[2 4]), '-bo', 'MarkerSize', 5);
fig1_payload = plot3(ax3, drone1_atti(1,[5 6]), drone1_atti(2,[5 6]), drone1_atti(3,[5 6]), '-k',  'Linewidth', 3);
fig1_shadow  = plot3(ax3, 0, 0, 0, 'xk', 'LineWidth', 3);
hold(ax3, 'off');

%% Figure(2D Data)
fig2 = figure('pos', [800 550 800 450]); clf(fig2)
tiledlayout(fig2, 2, 3, "Padding","compact", "TileSpacing","compact");

nexttile(1); ax2(1)=gca; title('\phi [deg]'); grid on; hold on; h(1)=animatedline('Marker','.', 'LineStyle','none');
nexttile(2); ax2(2)=gca; title('\theta [deg]'); grid on; hold on; h(2)=animatedline('Marker','.', 'LineStyle','none');
nexttile(3); ax2(3)=gca; title('\psi [deg]'); grid on; hold on; h(3)=animatedline('Marker','.', 'LineStyle','none');
nexttile(4); ax2(4)=gca; title('x [m]');      grid on; hold on; h(4)=animatedline('Marker','.', 'LineStyle','none');
nexttile(5); ax2(5)=gca; title('y [m]');      grid on; hold on; h(5)=animatedline('Marker','.', 'LineStyle','none');
nexttile(6); ax2(6)=gca; title('zdot [m/s]');    grid on; hold on; h(6)=animatedline('Marker','.', 'LineStyle','none');

for k=1:6, xlim(ax2(k), [0 simulationTime]); end

%% MP4 setting
video_filename = 'drone_animation_test.mp4';
v = VideoWriter(video_filename, 'MPEG-4');

fps_video = 25;
v.FrameRate = fps_video;
frame_skip = 4;

video_filename_2d = 'drone_plots_2d_test.mp4';
v2 = VideoWriter(video_filename_2d, 'MPEG-4');
v2.FrameRate = fps_video;

open(v);
open(v2);
%% ===== SIMULATION LOOP =====
for i = 1:N

    next = next + dt;
    while toc(t0) < next
        pause(dt/10);
    end

    if mod(i, frame_skip) ~= 0
        continue;
    end

    drone1_state = stateHistory_test(i+1, :).';
    ti = t(i+1);

    % 3D Plot
    wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
    drone1_world = wHb * drone1_body;
    drone1_atti  = drone1_world(1:3, :);

    set(fig1_ARM13,   'XData',drone1_atti(1,[1 3]), 'YData',drone1_atti(2,[1 3]), 'ZData',drone1_atti(3,[1 3]));
    set(fig1_ARM24,   'XData',drone1_atti(1,[2 4]), 'YData',drone1_atti(2,[2 4]), 'ZData',drone1_atti(3,[2 4]));
    set(fig1_payload, 'XData',drone1_atti(1,[5 6]), 'YData',drone1_atti(2,[5 6]), 'ZData',drone1_atti(3,[5 6]));
    set(fig1_shadow,  'XData',drone1_state(1),      'YData',drone1_state(2),      'ZData',0);

    % 2D Plot
    addpoints(h(1), ti, drone1_state(7)*R2D);
    addpoints(h(2), ti, drone1_state(8)*R2D);
    addpoints(h(3), ti, drone1_state(9)*R2D);
    addpoints(h(4), ti, drone1_state(1));
    addpoints(h(5), ti, drone1_state(2));
    addpoints(h(6), ti, drone1_state(3)); %6

    drawnow;

    frame1 = getframe(fig1);
    writeVideo(v, frame1);

    frame2 = getframe(fig2);
    writeVideo(v2, frame2);

    if (drone1_state(3) >= 0)
        msgbox('Crashed!!', 'Error', 'error');
        break;
    end
end

close(v);
close(v2);

%% command plot setting
phi_des   = 10.0;   % [deg]
theta_des = 10.0;   % [deg]
psi_des   = 10.0;   % [deg]
zdot_des  = -1.0;   % [m/s]

numPoints = size(stateHistory_test, 1);
t = (0:numPoints-1)' * dt;

phi_cmd_vec   = ones(numPoints, 1) * phi_des;
theta_cmd_vec = ones(numPoints, 1) * theta_des;
psi_cmd_vec   = ones(numPoints, 1) * psi_des;
zdot_cmd_vec  = ones(numPoints, 1) * zdot_des;

%% 4. 2x2 Subplot 생성
fig = figure('Name', 'State Variables vs. Commands', 'pos', [100 100 800 600]);
sgtitle('Drone State vs. Command Inputs', 'FontSize', 14, 'FontWeight', 'bold');

% Plot 1: Roll (phi)
subplot(2, 2, 1);
plot(t, stateHistory_test(:, 7) * R2D, 'r', 'LineWidth', 1.5); % stateHistory의 7번째 열 (phi)
hold on;
plot(t, phi_cmd_vec, 'b--', 'LineWidth', 1.5);
grid on;
title('Roll (\phi) History');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Roll (\phi)', 'Command', 'Location', 'southeast');

% Plot 2: Pitch (theta)
subplot(2, 2, 2);
plot(t, stateHistory_test(:, 8) * R2D, 'r', 'LineWidth', 1.5); % stateHistory의 8번째 열 (theta)
hold on;
plot(t, theta_cmd_vec, 'b--', 'LineWidth', 1.5);
grid on;
title('Pitch (\theta) History');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Pitch (\theta)', 'Command', 'Location', 'southeast');

% Plot 3: Yaw (psi)
subplot(2, 2, 3);
plot(t, stateHistory_test(:, 9) * R2D, 'r', 'LineWidth', 1.5); % stateHistory의 9번째 열 (psi)
hold on;
plot(t, psi_cmd_vec, 'b--', 'LineWidth', 1.5);
grid on;
title('Yaw (\psi) History');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Yaw (\psi)', 'Command', 'Location', 'southeast');

% Plot 4: Z-dot (Vertical Speed)
subplot(2, 2, 4);
plot(t, stateHistory_test(:, 6), 'r', 'LineWidth', 1.5); % stateHistory의 6번째 열 (Z-dot)
hold on;
plot(t, zdot_cmd_vec, 'b--', 'LineWidth', 1.5);
grid on;
title('Vertical Speed (Z-dot) History');
xlabel('Time (s)');
ylabel('Speed (m/s)');
legend('Z-dot', 'Command', 'Location', 'southeast');
saveas(fig, 'drone_state_results.png');
