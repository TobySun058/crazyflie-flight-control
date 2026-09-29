clear; clc; close all;

%% Altitude Control via LQR-PI

% Parameters
m = 0.033;
g = 9.81;
A_thrust = 0.091492681;
B_thrust = 0.067673604;

% Hover
T_hover_total = m * g;

% Function
f_hover = @(PWM) A_thrust * PWM.^2 + B_thrust * PWM - T_hover_total / 4;

% Initial guess
PWM_guess = 0.5;

% Solve function
PWM_hover = fsolve(f_hover, PWM_guess);

% Augmented State-space model with integral action
A = [0 1 0;
    0 0 0;
    -1 0 0];

B = [0;
    1/m;
    0];

B_2 = [0;
    0;
    1];

C = [1 0 0];

D = 0;

% LQR weights
Q = diag([20 5 15]);
R_values = [1 2 5 10 20 50 100 200 500 1000 2000 5000 10000];

% Open-loop system for frequency response
G = ss(A, B, eye(3), zeros(3,1));

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

legend_entries = cell(length(R_values),1);

fprintf('\nLQR gains and margins for each R:\n');

for i = 1:length(R_values)
    R = R_values(i);

    % LQR gain
    K = lqr(A, B, Q, R);
    k1 = K(1);
    k2 = K(2);
    k3 = K(3);

    fprintf('\nFor R = %.6f\n', R);
    fprintf('k1 = %.6f\n', k1);
    fprintf('k2 = %.6f\n', k2);
    fprintf('k3 = %.6f\n', k3);

    % Transfer Function
    L = K * G;

    % Nyquist Plot
    figure(1);
    nyquist(L);
    xlim([-150 5]);
    ylim([-20 20]);

    % Bode Plot
    figure(2);
    bode(L);

    % Gain and phase margins
    [GM, PM, ~ , ~] = margin(L);
    fprintf('Gain Margin = %.6f dB\n', abs(20*log10(GM)));
    fprintf('Phase Margin = %.6f deg\n', PM);

    legend_entries{i} = sprintf('R = %.2g', R);
end

figure(1);
legend(legend_entries, 'Location', 'best');

figure(2);
legend(legend_entries, 'Location', 'best');

% Choose final R based on the plot comparison
R = 1000;

% LQR gain
K = lqr(A, B, Q, R);
k1 = K(1);
k2 = K(2);
k3 = K(3);


fprintf('\nChosen Final R = %.6f\n', R);

fprintf('\nLQR gains:\n');
fprintf('k1 = %.6f\n', k1);
fprintf('k2 = %.6f\n', k2);
fprintf('k3 = %.6f\n', k3);

% Closed-loop system
Acl = A - B*K;
Bcl = B_2;
Ccl = eye(3);
Dcl = zeros(3,1);
sys_cl = ss(Acl, Bcl, Ccl, Dcl);

% Transfer Function
G_final = ss(A, B, eye(3), 0);
L_final = K*G_final;


% Step response for 1m command
t = linspace(0,10,1000)';
r = ones(size(t)); % 1 m referenc

x = lsim(sys_cl, r, t);  % simulate states
z = x(:,1);   % altitude state

% Plot altitude response
figure;
plot(t, z, 'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Closed-Loop Step Response to 1 m Altitude Command');

%% Connect mixer design to controller output

%  Parameters
k = 0.005964552;
l = 0.046;
PWM_min = 0.0;
PWM_max = 1.0;

% Compute controller thrust command
deltaT = -(x * K.');
T_cmd = T_hover_total + deltaT;

% History of thurusts, PWM, and saturation
T_motor_hist = zeros(length(t),4);
PWM_hist = zeros(length(t),4);
alpha_hist = zeros(length(t),1);

for i = 1:length(t)
    [T_motor_i, PWM_i, alpha_i] = wrench_to_pwm( ...
        T_cmd(i), 0, 0, 0, ...
        A_thrust, B_thrust, k, l, PWM_min, PWM_max);

    T_motor_hist(i,:) = T_motor_i.';
    PWM_hist(i,:) = PWM_i.';
    alpha_hist(i) = alpha_i;
end

% Plot total thrust command
figure;
plot(t, T_cmd, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Total thrust command (N)');
title('Controller Output: Total Thrust Command');

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