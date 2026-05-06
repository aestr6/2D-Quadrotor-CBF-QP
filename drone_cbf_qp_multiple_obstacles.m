clear
close all
clc

set(0, 'defaulttextInterpreter', 'latex')
set(0, 'defaultAxesTickLabelInterpreter', 'latex')
set(0, 'defaultLegendInterpreter', 'latex')
set(0, 'DefaultTextFontSize', 13)
set(0, 'defaultAxesFontSize', 13)
set(0, 'defaultLegendFontSize', 13)
set(0, 'defaultLineLineWidth', 2)


% system model
g = 9.81;                   % gravity
m = 2;                      % mass %kg
vd = 0.5;                   % desired speed
ti = 20;                    % duration of ascention
ca = 2.0;                   % max acceleration fraction of g
phi_max = 30 * pi / 180;  % max pitch
R_obs = 0.75;                % obstacle safety distance

% weights
p_sc = 1e-5;             % small control penalty
k_p = 10.0;              % nominal proportional gain
k_d = 5.0;               % nominal derivative gain
gamma1 = 4.0;            % obstacle barrier function derivative gain
gamma2 = 4.0;            % obstacle barrier function proportional gain
beta1 = 3.0;             % height barrier function derivative gain
beta2 = 3.0;             % height barrier function proportional gain



% simulation data
T = 60.0;                  % final time
Ts = 0.01;                 % sample time

t = 0:Ts:T;
N = numel(t);

% initial condition
x0 = [0; 0; 0; 0];          % [x; y; dx, dy]

% Obstacle Positions
pos_obs1 = [8; vd * ti];    % [x; y]
pos_obs2 = [8; vd * ti + 1];    % [x; y]
pos_obs3 = [0; 5];    % [x; y]
pos_obs4 = [8 + 2; vd * ti - 1];    % [x; y]
pos_obs5 = [8 + 2.75; vd * ti - 0.25];    % [x; y]
pos_obs6 = [8 + 3.25; vd * ti + 0.35];    % [x; y]

% pos_obs3 = pos_obs1;
% pos_obs4 = pos_obs1;
% pos_obs5 = pos_obs1;
% pos_obs6 = pos_obs1;

% u = [ux, uy]
% ux = (T/m) sin(phi)
% uy = (T/m) cos(phi)
% T = m (ux^2 + uy^2)
% phi = arctan(ux/uy)

% simulate
out = simulate_acc(t, x0, g, m, ca, p_sc, phi_max, vd, ti, k_p, k_d, gamma1, gamma2, beta1, beta2, pos_obs1, R_obs, pos_obs2, pos_obs3, pos_obs4, pos_obs5, pos_obs6);

x_des_history = zeros(1, N);
y_des_history = zeros(1, N);

for k = 1:N
    [des, ~] = des_traj(x0, t(k), vd, ti);
    x_des_history(k) = des(1);
    y_des_history(k) = des(2);
end

Thrust_mag = sqrt(out.u(1, :).^2 + out.u(2, :).^2);
Pitch_deg = atan2(out.u(1, :), out.u(2, :)) * (180 / pi);
T_max = m * ca * g;
phi_max_deg = phi_max * (180 / pi);

% CBF Drone Performance
figure(1)
set(gcf, 'Position', [100, 100, 800, 1000])
% X Position
subplot(3,2,1)
plot(t, out.x(1, :), 'b-', t, x_des_history, 'k--')
grid on
xlabel('$t$ [s]')
ylabel('$x$ [m]')
title('X-Position Tracking')
legend('Actual $x$', 'Desired $x_d$', 'Location', 'best')

% Y Position
subplot(3,2,2)
plot(t, out.x(2, :), 'b-', t, y_des_history, 'k--')
grid on
xlabel('$t$ [s]')
ylabel('$y$ [m]')
title('Y-Position Tracking')
legend('Actual $y$', 'Desired $y_d$', 'Location', 'best')

% Velocities
subplot(3,2,3)
plot(t, out.x(3, :), '-', t, out.x(4, :), '-')
grid on
xlabel('$t$ [s]')
ylabel('Velocity [m/s]')
title('X and Y Velocities')
legend('$\dot{x}$ (Lateral)', '$\dot{y}$ (Vertical)', 'Location', 'best')

% Ux and Uy
subplot(3,2,4)
plot(t, out.u(1, :), '-', t, out.u(2, :), '-')
grid on
xlabel('$t$ [s]')
ylabel('Input [N]')
title('Alternative Control Inputs')
legend('$u_x$', '$u_y$', 'Location', 'best')

% Actual Thrust
subplot(3,2,5)
plot(t, Thrust_mag, 'b-', t, T_max*ones(size(t)), 'r--')
grid on
xlabel('$t$ [s]')
ylabel('Thrust [N]')
title('Total Thrust Magnitude')
legend('$T_{total}$', '$T_{\max}$', 'Location', 'best')

% Pitch Angle
subplot(3,2,6)
plot(t, Pitch_deg, 'b-', ...
     t, phi_max_deg*ones(size(t)), 'r--', ...
     t, -phi_max_deg*ones(size(t)), 'r--')
grid on
xlabel('$t$ [s]')
ylabel('Pitch [deg]')
title('Pitch Angle')
legend('$\phi$', '$\pm \phi_{\max}$', 'Location', 'southwest')

set(0, 'DefaultTextFontSize', 22)
set(0, 'defaultAxesFontSize', 22)
set(0, 'defaultLegendFontSize', 22)
% Trajectory Plot
figure(2)
hold on; grid on;
plot(x_des_history, y_des_history, 'k--', 'LineWidth', 2, 'DisplayName', 'Desired Trajectory');
plot(out.x(1, :), out.x(2, :), 'b-', 'LineWidth', 2, 'DisplayName', 'CBF-QP Trajectory');
plot(pos_obs1(1), pos_obs1(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Obstacle');
plot(pos_obs2(1), pos_obs2(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
plot(pos_obs3(1), pos_obs3(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
plot(pos_obs4(1), pos_obs4(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
plot(pos_obs5(1), pos_obs5(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
plot(pos_obs6(1), pos_obs6(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
plot(out.x(1, 1), out.x(2, 1), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(out.x(1, end), out.x(2, end), 'mo', 'MarkerFaceColor', 'm', 'DisplayName', 'End');
txt_offset = 0.4;
text(pos_obs1(1) + txt_offset, pos_obs1(2) + txt_offset, '1', 'FontSize', 12, 'FontWeight', 'bold');
text(pos_obs2(1) + txt_offset, pos_obs2(2) + txt_offset, '2', 'FontSize', 12, 'FontWeight', 'bold');
text(pos_obs3(1) + txt_offset, pos_obs3(2), '3', 'FontSize', 12, 'FontWeight', 'bold');
text(pos_obs4(1) + txt_offset, pos_obs4(2), '4', 'FontSize', 12, 'FontWeight', 'bold');
text(pos_obs5(1) + txt_offset, pos_obs5(2), '5', 'FontSize', 12, 'FontWeight', 'bold');
text(pos_obs6(1) + txt_offset, pos_obs6(2), '6', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('$x$ [m]')
ylabel('$y$ [m]')
title('2D Quadrotor Trajectory')
legend('Location', 'best')
axis equal; 
xlim([-1, 22]);
hold off;

figure(3)
hold on; grid on;
plot(t, out.h,  'DisplayName', 'Obstacle 1')
plot(t, out.h_obs2,  'DisplayName', 'Obstacle 2');
plot(t, out.h_obs3,  'DisplayName', 'Obstacle 3');
plot(t, out.h_obs4,  'DisplayName', 'Obstacle 4');
plot(t, out.h_obs5,  'DisplayName', 'Obstacle 5');
plot(t, out.h_obs6,  'DisplayName', 'Obstacle 6');
grid on
xlabel('$t$ [s]')
ylabel('$h_{obs}(\mathbf{x})$')
title('Obstacle Avoidance CBF')
legend('Location', 'best', 'NumColumns', 2)
hold off;

figure(4)
hold on; grid on;
plot(t, out.h_obs_ineq,  'DisplayName', 'Obstacle 1');
plot(t, out.h_obs_ineq2, 'DisplayName', 'Obstacle 2');
plot(t, out.h_obs_ineq3, 'DisplayName', 'Obstacle 3');
plot(t, out.h_obs_ineq4, 'DisplayName', 'Obstacle 4');
plot(t, out.h_obs_ineq5, 'DisplayName', 'Obstacle 5');
plot(t, out.h_obs_ineq6, 'DisplayName', 'Obstacle 6');
% grid on
xlabel('$t$ [s]')
ylabel('$\ddot h_{obs}(\mathbf{x, u}) + \gamma_1 \dot h_{obs}(\mathbf{x}) + \gamma_2 h_{obs}(\mathbf{x})$')
title('Obstacle Avoidance CBF Inequality')
legend('Location', 'best', 'NumColumns', 2)
hold off;

figure(5)
plot(t, out.h_y_ineq, 'b-')
grid on
xlabel('$t$ [s]')
ylabel('$\ddot h_y(\mathbf{x, u}) + \beta_1 \dot h_y(\mathbf{x}) + \beta_2 h_y(\mathbf{x})$')
title('Height CBF Inequality')


function out = simulate_acc(t, x0, g, m, ca, p_sc, phi_max, vd, ti, k_p, k_d, gamma1, gamma2, beta1, beta2, pos_obs1, R_obs, pos_obs2, pos_obs3, pos_obs4, pos_obs5, pos_obs6)

    N = numel(t);

    x = zeros(4, N);
    u = zeros(2, N);
    h = zeros(1, N);
    h_obs_ineq = zeros(1, N);
    h_y_ineq = zeros(1, N);
    h_obs_ineq2 = zeros(1, N);
    h_obs2 = zeros(1, N);
    h_obs_ineq3 = zeros(1, N);
    h_obs3 = zeros(1, N);
    h_obs_ineq4 = zeros(1, N);
    h_obs4 = zeros(1, N);
    h_obs_ineq5 = zeros(1, N);
    h_obs5 = zeros(1, N);
    h_obs_ineq6 = zeros(1, N);
    h_obs6 = zeros(1, N);
    exitflag = zeros(1, N);

    x(:, 1) = x0;

    opts_ode = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
    opts_qp = optimoptions('quadprog', 'Display', 'off');

    for k = 1:N-1
        [u(:, k), h(k), h_obs_ineq(k), h_y_ineq(k), h_obs2(k), h_obs_ineq2(k), h_obs3(k), h_obs_ineq3(k), h_obs4(k), ...
            h_obs_ineq4(k), h_obs5(k), h_obs_ineq5(k), h_obs6(k), h_obs_ineq6(k), exitflag(k)] = acc_qp(opts_qp, t(k), x(:, k), ...
            x0, g, m, ca, p_sc, phi_max, vd, ti, k_p, k_d, gamma1, gamma2, beta1, beta2, pos_obs1, R_obs, ...
            pos_obs2, pos_obs3, pos_obs4, pos_obs5, pos_obs6);

        uk = u(:,k);
        dyn = @(tau, xk) acc_rhs(xk, uk, g, m);
        [~, xsol] = ode45(dyn, [t(k), t(k+1)], x(:, k), opts_ode);
        x(:, k+1) = xsol(end, :)';
    end

    [u(:, N), h(N), h_obs_ineq(N), h_y_ineq(N), h_obs2(N), h_obs_ineq2(N), h_obs3(N), h_obs_ineq3(N), h_obs4(N), h_obs_ineq4(N), ...
        h_obs5(N), h_obs_ineq5(N), h_obs6(N), h_obs_ineq6(N), exitflag(N)] = acc_qp(opts_qp, t(N), x(:, N), ...
            x0, g, m, ca, p_sc, phi_max, vd, ti, k_p, k_d, gamma1, gamma2, beta1, beta2, pos_obs1, R_obs, ...
            pos_obs2, pos_obs3, pos_obs4, pos_obs5, pos_obs6);

    out.x = x;
    out.u = u;
    out.h = h;
    out.h_obs_ineq = h_obs_ineq;
    out.h_obs2 = h_obs2;
    out.h_obs_ineq2 = h_obs_ineq2;
    out.h_obs3 = h_obs3;
    out.h_obs_ineq3 = h_obs_ineq3;
    out.h_obs4 = h_obs4;
    out.h_obs_ineq4 = h_obs_ineq4;
    out.h_obs5 = h_obs5;
    out.h_obs_ineq5 = h_obs_ineq5;
    out.h_obs6 = h_obs6;
    out.h_obs_ineq6 = h_obs_ineq6;
    out.h_y_ineq = h_y_ineq;
    out.exitflag = exitflag;
end

function [u, h_obs, h_obs_ineq, h_y_ineq, h_obs2, h_obs_ineq2, h_obs3, h_obs_ineq3, h_obs4, h_obs_ineq4, h_obs5, h_obs_ineq5, h_obs6, h_obs_ineq6, exitflag] = acc_qp(opts_qp, t_k, x, x0, g, m, ca, p_sc, phi_max, vd, ti, k_p, k_d, gamma1, gamma2, beta1, beta2, pos_obs1, R_obs, pos_obs2, pos_obs3, pos_obs4, pos_obs5, pos_obs6)
    
    % Desired trajectory and velocity
    [x_des, dx_des] = des_traj(x0, t_k, vd, ti);
    
    % Nominal PD controller
    u_x_nom = m * (-k_p*(x(1) - x_des(1)) - k_d*(x(3) - dx_des(1)));
    u_y_nom = m * (-k_p*(x(2) - x_des(2)) - k_d*(x(4) - dx_des(2)) + g);
    
    % Exponential CBF Data
    dx_obs = x(1) - pos_obs1(1);
    dy_obs = x(2) - pos_obs1(2);
    
    h_obs = 0.5 * (dx_obs^2 + dy_obs^2) - 0.5 * R_obs^2;
    h_obs_dot = dx_obs * x(3) + dy_obs * x(4);
    h_obs_ddot = x(3)^2 + x(4)^2 - dy_obs * g;

    % Aw = b, w = [ux, uy]
    A_obs = [-dx_obs/m, -dy_obs/m];
    b_obs = h_obs_ddot + gamma1 * h_obs_dot + gamma2 * h_obs;

    A_y = [0, -1.0/m];
    b_y = -g + beta1 * x(4) + beta2 * x(2);

    % Extra obstacles
    % Obstacle 2
    dx_obs2 = x(1) - pos_obs2(1);
    dy_obs2 = x(2) - pos_obs2(2);
    h_obs2 = 0.5 * (dx_obs2^2 + dy_obs2^2) - 0.5 * R_obs^2;
    h_obs_dot2 = dx_obs2 * x(3) + dy_obs2 * x(4);
    h_obs_ddot2 = x(3)^2 + x(4)^2 - dy_obs2 * g;
    A_obs2 = [-dx_obs2/m, -dy_obs2/m];
    b_obs2 = h_obs_ddot2 + gamma1 * h_obs_dot2 + gamma2 * h_obs2;

    % Obstacle 3
    dx_obs3 = x(1) - pos_obs3(1);
    dy_obs3 = x(2) - pos_obs3(2);
    h_obs3 = 0.5 * (dx_obs3^2 + dy_obs3^2) - 0.5 * R_obs^2;
    h_obs_dot3 = dx_obs3 * x(3) + dy_obs3 * x(4);
    h_obs_ddot3 = x(3)^2 + x(4)^2 - dy_obs3 * g;
    A_obs3 = [-dx_obs3/m, -dy_obs3/m];
    b_obs3 = h_obs_ddot3 + gamma1 * h_obs_dot3 + gamma2 * h_obs3;

    % Obstacle 4
    dx_obs4 = x(1) - pos_obs4(1);
    dy_obs4 = x(2) - pos_obs4(2);
    h_obs4 = 0.5 * (dx_obs4^2 + dy_obs4^2) - 0.5 * R_obs^2;
    h_obs_dot4 = dx_obs4 * x(3) + dy_obs4 * x(4);
    h_obs_ddot4 = x(3)^2 + x(4)^2 - dy_obs4 * g;
    A_obs4 = [-dx_obs4/m, -dy_obs4/m];
    b_obs4 = h_obs_ddot4 + gamma1 * h_obs_dot4 + gamma2 * h_obs4;

    % Obstacle 5
    dx_obs5 = x(1) - pos_obs5(1);
    dy_obs5 = x(2) - pos_obs5(2);
    h_obs5 = 0.5 * (dx_obs5^2 + dy_obs5^2) - 0.5 * R_obs^2;
    h_obs_dot5 = dx_obs5 * x(3) + dy_obs5 * x(4);
    h_obs_ddot5 = x(3)^2 + x(4)^2 - dy_obs5 * g;
    A_obs5 = [-dx_obs5/m, -dy_obs5/m];
    b_obs5 = h_obs_ddot5 + gamma1 * h_obs_dot5 + gamma2 * h_obs5;

    % Obstacle 6
    dx_obs6 = x(1) - pos_obs6(1);
    dy_obs6 = x(2) - pos_obs6(2);
    h_obs6 = 0.5 * (dx_obs6^2 + dy_obs6^2) - 0.5 * R_obs^2;
    h_obs_dot6 = dx_obs6 * x(3) + dy_obs6 * x(4);
    h_obs_ddot6 = x(3)^2 + x(4)^2 - dy_obs6 * g;
    A_obs6 = [-dx_obs6/m, -dy_obs6/m];
    b_obs6 = h_obs_ddot6 + gamma1 * h_obs_dot6 + gamma2 * h_obs6;

    % QP Formulation (Decision variables: [ux, uy])
    H = diag([p_sc, p_sc]);
    f = [-p_sc*u_x_nom; -p_sc*u_y_nom];
    
    % Constraints: [Obstacle CBF; Height CBF; Pitch(+); Pitch(-); Thrust(+); Thrust(-); Min Y Thrust]
    A = [ A_obs; ...
          A_y; ...
          A_obs2; ...
          A_obs3; ...
          A_obs4; ...
          A_obs5; ...
          A_obs6; ...
             1.0, -tan(phi_max); ...
            -1.0, -tan(phi_max); ...
             1.0,             1.0; ...
            -1.0,             1.0; ...
             0.0,            -1.0];
             
    b = [ b_obs; ...   
          b_y; ...
          b_obs2; ... 
          b_obs3; ... 
          b_obs4; ... 
          b_obs5; ... 
          b_obs6; ... 
          0; ...
          0; ...
          m*ca*g; ...
          m*ca*g; ...
          0.0];
          
    lb = [-inf; -inf];
    ub = [ inf;  inf];
    
    [w, ~, exitflag] = quadprog(H, f, A, b, [], [], lb, ub, [], opts_qp);
    
    if isempty(w) || exitflag <= 0
        u = [u_x_nom; u_y_nom];
        h_y_ineq = b_y + A_y(1) * u_x_nom + A_y(2) * u_y_nom;
        h_obs_ineq = b_obs + A_obs(1) * u_x_nom + A_obs(2) * u_y_nom;
        h_obs_ineq2 = b_obs2 + A_obs2(1) * u_x_nom + A_obs2(2) * u_y_nom;
        h_obs_ineq3 = b_obs3 + A_obs3(1) * u_x_nom + A_obs3(2) * u_y_nom;
        h_obs_ineq4 = b_obs4 + A_obs4(1) * u_x_nom + A_obs4(2) * u_y_nom;
        h_obs_ineq5 = b_obs5 + A_obs5(1) * u_x_nom + A_obs5(2) * u_y_nom;
        h_obs_ineq6 = b_obs6 + A_obs6(1) * u_x_nom + A_obs6(2) * u_y_nom;
    else
        u = [w(1); w(2)];
        h_y_ineq = b_y - A_y(1) * w(1) - A_y(2) * w(2);
        h_obs_ineq = b_obs - A_obs(1) * w(1) - A_obs(2) * w(2);
        h_obs_ineq2 = b_obs2 - A_obs2(1) * w(1) - A_obs2(2) * w(2);
        h_obs_ineq3 = b_obs3 - A_obs3(1) * w(1) - A_obs3(2) * w(2);
        h_obs_ineq4 = b_obs4 - A_obs4(1) * w(1) - A_obs4(2) * w(2);
        h_obs_ineq5 = b_obs5 - A_obs5(1) * w(1) - A_obs5(2) * w(2);
        h_obs_ineq6 = b_obs6 - A_obs6(1) * w(1) - A_obs6(2) * w(2);
    end
end


function dx = acc_rhs(x, u, g, m)

    dx = [x(3); ...
         x(4); ...
          u(1)/m; ...
          u(2)/m-g];
end

function [des, ddes] = des_traj(x0, t, vd, t_i)
    if t < t_i
        des = [x0(1); x0(2) + vd * t; 0; vd];
        ddes = [0; vd; 0; 0];
    else
        des = [x0(1) + vd * (t - t_i); x0(2) + vd * t_i ; vd; 0];
        ddes = [vd; 0; 0; 0];
    end
end