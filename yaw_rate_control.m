clear; clc; close all;

%% Yaw Rate Control via LQR

% Parameters
m = 0.033;
g = 9.81;
A_thrust = 0.091492681;
B_thrust = 0.067673604;

% Yaw inertia
Jzz = 29.261652e-6;

% Hover
T_hover_total = m * g;

% Function
f_hover = @(PWM) A_thrust * PWM.^2 + B_thrust * PWM - T_hover_total / 4;

% Initial guess
PWM_guess = 0.5;

% Solve function
options = optimoptions('fsolve','Display','off');
PWM_hover = fsolve(f_hover, PWM_guess, options);

% Reduced State-space model
rad2deg = 180/pi;

A = 0;
B = rad2deg / Jzz;
C = 1;
D = 0;

% LQR weights
Q = 1;
R_values = [ 1 1e1 1e3 1e5 1e7 1e9 1e11 1e13 1e15 1e17 1e19];


% Open-loop system for frequency response
G = ss(A, B, C, D);

% Nyquist Plot
figure;
hold on;
grid on;
title('Nyquist Plot');

% Bode Plot
figure;
hold on;
grid on;
title('Bode Plot');

% Legend List
legend_entries = cell(length(R_values),1);

for i = 1:length(R_values)
    R = R_values(i);

    % LQR gain
    K = lqr(A, B, Q, R);

    fprintf('\nFor R = %.6e\n', R);
    fprintf('K = %.10f\n', K);

    % Transfer Function
    L = K * G;

    % Nyquist Plot
    figure(1);
    nyquist(L);
    xlim([-10 2]);
    ylim([-20 20]);

    % Bode Plot
    figure(2);
    bode(L);

    % Gain and phase margins
    [GM, PM, ~ , ~] = margin(L);
    fprintf('Gain Margin = %.6f dB\n', abs(20*log10(GM)));
    fprintf('Phase Margin = %.6f deg\n', PM);

    legend_entries{i} = sprintf('R = %.0e', R);
end

figure(1);
legend(legend_entries, 'Location', 'best');

figure(2);
legend(legend_entries, 'Location', 'best');

R = 1e12;

% LQR gain
K = lqr(A, B, Q, R);

fprintf('\n Chosen Final R = %.6e\n', R);

fprintf('\nLQR gain:\n');
fprintf('K = %.10f\n', K);

% Closed-loop system
Acl = A - B * K;
Bcl = 0;
Ccl = 1;
Dcl = 0;
sys_cl = ss(Acl, Bcl, Ccl, Dcl);

% Transfer Function
G_final = ss(A, B, C, D);
L_final = K * G_final;

% Initial response for 100 deg/s yaw rate
t = linspace(0,10,1000)';
x0 = 100; % 100 deg/s initial yaw rate

[y,t,x] = initial(sys_cl, x0, t);
r_rate = x(:,1); % yaw rate state

% Plot yaw rate response
figure;
plot(t, r_rate, 'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');
title('Closed-Loop Response from 100 deg/s Initial Yaw Rate');

%% Connect mixer design to controller output

%  Parameters
k = 0.005964552;
l = 0.046;
PWM_min = 0.0;
PWM_max = 1.0;

% Yaw moment command
deltaN = -(x * K.');

% History of thurusts, PWM, and saturation
T_motor_hist = zeros(length(t),4);
PWM_hist = zeros(length(t),4);
alpha_hist = zeros(length(t),1);

for i = 1:length(t)
    [T_motor_i, PWM_i, alpha_i] = mixer_pwm_command( ...
        T_hover_total, 0, 0, deltaN(i), ...
        A_thrust, B_thrust, k, l, PWM_min, PWM_max);

    T_motor_hist(i,:) = T_motor_i.';
    PWM_hist(i,:) = PWM_i.';
    alpha_hist(i) = alpha_i;
end

% Plot yawing moment command
figure;
plot(t, deltaN, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Yawing moment command (N*m)');
title('Controller Output: Yawing Moment Command');

% Plot motor thrusts from mixer
figure;
plot(t, T_motor_hist(:,1), 'LineWidth', 1.5); hold on;
plot(t, T_motor_hist(:,2), '--', 'LineWidth', 1.2);
plot(t, T_motor_hist(:,3), '-.', 'LineWidth', 1.2);
plot(t, T_motor_hist(:,4), ':', 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Motor thrust (N)');
title('Motor Thrust Commands from Mixer');
legend('T_1','T_2','T_3','T_4');

% Plot PWM responses
figure;
plot(t, PWM_hist(:,1), 'LineWidth', 1.5); hold on;
plot(t, PWM_hist(:,2), '--', 'LineWidth', 1.2);
plot(t, PWM_hist(:,3), '-.', 'LineWidth', 1.2);
plot(t, PWM_hist(:,4), ':', 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('PWM');
title('PWM Signal Behavior from Mixer');
legend('PWM_1','PWM_2','PWM_3','PWM_4');
