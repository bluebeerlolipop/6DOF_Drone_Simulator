addpath('./lib');
load('stateHistory.mat');

initState = stateHistory(1,:).';

%% DEFINE
R2D = 180/pi;
D2R = pi/180;

%% Simulation time
simulationTime = 2;
dt = 0.01;
N  = floor(simulationTime/dt);
t  = (0:N)*dt;

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

%% ===== SIMULATION LOOP =====
for i = 1:N
    drone1_state = stateHistory(i+1, :).';
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
    addpoints(h(6), ti, drone1_state(6));

    drawnow limitrate
    pause(dt)

    if (drone1_state(3) >= 0)
        msgbox('Crashed!!', 'Error', 'error');
        break;
    end
end
