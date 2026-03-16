clear; clc; close all;

%% Yaw Rate Control via LQR

% Parameters
m = 0.033;
g = 9.81;
A_thrust = 0.091492681;
B_thrust = 0.067673604;

% Yaw inertia from your J matrix
Jzz = 29.261652e-6;

% Hover
T_hover_total = m * g;

% Function
f_hover = @(PWM) A_thrust * PWM.^2 + B_thrust * PWM - T_hover_total / 4;

% Initial guess
PWM_guess = 0.5;

% Solve using fsolve
options = optimoptions('fsolve','Display','off');
PWM_hover = fsolve(f_hover, PWM_guess, options);

fprintf('Hover PWM = %.6f\n', PWM_hover);

% Reduced State-spacemodel
% State: x = r_deg
% Input: yawing moment N
rad2deg = 180/pi;

A = 0;
B = rad2deg / Jzz;
C = 1;
D = 0;

% LQR weights
Q = 1;
R = 100000000000;

% LQR gain
K = lqr(A, B, Q, R);

fprintf('\nLQR gain:\n');
fprintf('K = %.6f\n', K);

% Closed-loop system
Acl = A - B*K;
Bcl = 0;
Ccl = 1;
Dcl = 0;
sys_cl = ss(Acl, Bcl, Ccl, Dcl);

% Transfer Function
G = ss(A, B, C, D);
L = K*G;

% Nyquist Plot
figure;
nyquist(L);
grid on;
title('Nyquist Plot');

% Bode Plot
figure;
bode(L);
grid on;
title('Bode Plot');

% Gain and phase margins
[GM, PM, ~ , ~] = margin(L);
fprintf('Gain Margin = %.6f dB\n', 20*log10(GM));
fprintf('Phase Margin = %.6f deg\n', PM);

% Initial response for 100 deg/s yaw rate
t = linspace(0,3,500)';
x0 = 100; % 100 deg/s initial yaw rate

[y,t,x] = initial(sys_cl, x0, t);
z = x(:,1); % yaw rate state

% Plot yaw rate response
figure;
plot(t, z, 'LineWidth',1.5);
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

% Compute controller yaw moment command
T_cmd = -(x * K.');

% History of thurusts, PWM, and saturation
T_motor_hist = zeros(length(t),4);
PWM_hist = zeros(length(t),4);
alpha_hist = zeros(length(t),1);

for i = 1:length(t)
    [T_motor_i, PWM_i, alpha_i] = mixer_pwm_command( ...
        T_hover_total, 0, 0, T_cmd(i), ...
        A_thrust, B_thrust, k, l, PWM_min, PWM_max);

    T_motor_hist(i,:) = T_motor_i.';
    PWM_hist(i,:) = PWM_i.';
    alpha_hist(i) = alpha_i;
end

% Plot yawing moment command
figure;
plot(t, T_cmd, 'LineWidth', 1.5);
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

% Estimate time to damp a 100 deg/s yaw rate
threshold = 0.02 * abs(x0); % 2 percent band
idx = find(abs(z) <= threshold, 1, 'first');

if ~isempty(idx)
    fprintf('Estimated time to damp 100 deg/s yaw rate = %.6f s\n', t(idx));
else
    fprintf('Yaw rate did not enter the 2 percent band in the simulation window.\n');
end