clear; clc; close all;

%% Altitude Control via LQR-PI

% Parameters
m = 0.033;
g = 9.81;
A_thrust = 0.091492681;
B_thrust = 0.067673604;

% Roll inertia from your J matrix
Jxx = 16.571710e-6;

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

% Augmented State-spacemodel with integral action
A = [ 0 1 0 0 0;
      0 0 g 0 0;
      0 0 0 1 0;
      0 0 0 0 0;
     -1 0 0 0 0 ];

B = [0;
     0;
     0;
     1/Jxx;
     0];

B_2 = [0;
       0;
       0;
       0;
       1];

C = [1 0 0 0 0];
D = 0;

% LQR weights
Q = diag([80 20 500 10 200]);
R = 1;

% LQR gain
K = lqr(A, B, Q, R);
k1 = K(1);
k2 = K(2);
k3 = K(3);
k4 = K(4);
k5 = K(5);

fprintf('\nLQR gains:\n');
fprintf('k1 = %.6f\n', k1);
fprintf('k2 = %.6f\n', k2);
fprintf('k3 = %.6f\n', k3);
fprintf('k4 = %.6f\n', k4);
fprintf('k5 = %.6f\n', k5);

% Closed-loop system
Acl = A - B*K;
Bcl = B_2;
Ccl = eye(5);
Dcl = zeros(5,1);
sys_cl = ss(Acl, Bcl, Ccl, Dcl);

% Transfer Function
G = ss(A, B, eye(5), 0);
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

% Step response for 1m command
t = linspace(0,5,500)';
r = ones(size(t)); % 1 m referenc

x = lsim(sys_cl, r, t);        % simulate states
z = x(:,1);                    % altitude state

% Plot altitude response
figure;
plot(t, z, 'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Lateral position y (m)');
title('Closed-Loop Step Response to 1 m Lateral Position Command');

%% Connect mixer design to controller output

%  Parameters
k = 0.005964552;
l = 0.046;
PWM_min = 0.0;
PWM_max = 1.0;

% Compute controller thrust command
deltaT = -(x * K.');
T_cmd = deltaT;

% History of thurusts, PWM, and saturation
T_motor_hist = zeros(length(t),4);
PWM_hist = zeros(length(t),4);
alpha_hist = zeros(length(t),1);

for i = 1:length(t)
    [T_motor_i, PWM_i, alpha_i] = mixer_pwm_command( ...
        T_hover_total, T_cmd(i), 0, 0, ...
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
ylabel('Rolling moment command (N*m)');
title('Controller Output: Rolling Moment Command');

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

% Estimate time to move forward by 1 meter
info = stepinfo(z, t, 1);
fprintf('Estimated settling time to reach 1 m = %.6f s\n', info.SettlingTime);