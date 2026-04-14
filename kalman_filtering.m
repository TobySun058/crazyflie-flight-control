clc; clear; close all;

%% Part A
% Continuous time state space model
A = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0];

B = zeros(4,1);

C = [1 0 0 0;
     0 1 0 0];

D = zeros(2,1);

% Sampling time
dt = 1;  

% Convert continuous system to discrete system using ZOH
sys_c = ss(A,B,C,D);
sys_d = c2d(sys_c, dt, 'zoh');

% Extract matrix
Ad = sys_d.A;
Cd = sys_d.C;

Q = eye(4); % Process noise matrix
R = 0.25 * eye(2); % Measurement noise matrix

% Initial values
xhat0 = [0; 0; 0; 0];
P0 = diag([1 1 25 25]);

% Measurements

Y1 = [4.2484 4.9309 6.3238 7.7615 7.8829;
      2.8829 5.7896 7.3837 8.7653 11.2713];

Y2 = [3.7683 4.7671 6.1210 6.0434 7.1375;
      2.7189 4.4936 7.1571 8.5460 10.2938];

%% Part B

% Trial 1
N1 = size(Y1,2);

xhat = xhat0;
P = P0;

x_pred1 = zeros(4,N1);
x_filt1 = zeros(4,N1);
innov1  = zeros(2,N1);

for k = 1:N1
    
    % Predict
    x_minus = Ad * xhat;
    P_minus = Ad * P * Ad' + Q;
    
    % Innovation
    y_tilde = Y1(:,k) - Cd * x_minus;
    
    % Kalman gain
    S = Cd * P_minus * Cd' + R;
    K = P_minus * Cd' / S;
    
    % Correct
    xhat = x_minus + K * y_tilde;
    P = (eye(4) - K * Cd) * P_minus;
    
    % Save
    x_pred1(:,k) = x_minus;
    x_filt1(:,k) = xhat;
    innov1(:,k) = y_tilde;
end

disp('Final estimated state for Trial 1:');
disp(x_filt1(:,end));


% Trial 2
N2 = size(Y2,2);

xhat = xhat0;
P = P0;

x_pred2 = zeros(4,N2);
x_filt2 = zeros(4,N2);
innov2  = zeros(2,N2);

for k = 1:N2
    
    % Predict
    x_minus = Ad * xhat;
    P_minus = Ad * P * Ad' + Q;
    
    % Innovation
    y_tilde = Y2(:,k) - Cd * x_minus;
    
    % Kalman gain
    S = Cd * P_minus * Cd' + R;
    K = P_minus * Cd' / S;
    
    % Correct
    xhat = x_minus + K * y_tilde;
    P = (eye(4) - K * Cd) * P_minus;
    
    % Save
    x_pred2(:,k) = x_minus;
    x_filt2(:,k) = xhat;
    innov2(:,k) = y_tilde;
end

disp('Final estimated state for Trial 2:');
disp(x_filt2(:,end));

% Plot 

% Measured positions, predicted positions, and corrected positions for
% Trial 1
t1 = 1:N1;


figure;
subplot(2,1,1);
plot(t1, Y1(1,:), 'ko-', 'LineWidth', 1.5); hold on;
plot(t1, x_pred1(1,:), 'bo-', 'LineWidth', 1.5);
plot(t1, x_filt1(1,:), 'ro-', 'LineWidth', 1.5);
grid on;
xlabel('Time step');
ylabel('x_1 (m)');
title('Trial 1 - North Position');
legend('Measured','Predicted','Filtered');

subplot(2,1,2);
plot(t1, Y1(2,:), 'ko-', 'LineWidth', 1.5); hold on;
plot(t1, x_pred1(2,:), 'bo-', 'LineWidth', 1.5);
plot(t1, x_filt1(2,:), 'ro-', 'LineWidth', 1.5);
grid on;
xlabel('Time step k');
ylabel('x_2 (m)');
title('Trial 1 - East Position');
legend('Measured','Predicted','Filtered');

% Innovation plot for Trial 1
figure;
plot(t1, innov1(1,:), 'bo-', 'LineWidth', 1.5); hold on;
plot(t1, innov1(2,:), 'ro-', 'LineWidth', 1.5);
yline(0,'k--');
grid on;
xlabel('Time step');
ylabel('Innovation');
title('Trial 1 Innovation');
legend('\tilde{y}_1','\tilde{y}_2');

% Measured positions, predicted positions, and corrected positions for
% Trial 2
t2 = 1:N2;

figure;
subplot(2,1,1);
plot(t2, Y2(1,:), 'ko-', 'LineWidth', 1.5); hold on;
plot(t2, x_pred2(1,:), 'bo-', 'LineWidth', 1.5);
plot(t2, x_filt2(1,:), 'ro-', 'LineWidth', 1.5);
grid on;
xlabel('Time step');
ylabel('x_1 (m)');
title('Trial 2 - North Position');
legend('Measured','Predicted','Filtered');

subplot(2,1,2);
plot(t2, Y2(2,:), 'ko-', 'LineWidth', 1.5); hold on;
plot(t2, x_pred2(2,:), 'bo-', 'LineWidth', 1.5);
plot(t2, x_filt2(2,:), 'ro-', 'LineWidth', 1.5);
grid on;
xlabel('Time step');
ylabel('x_2 (m)');
title('Trial 2 - East Position');
legend('Measured','Predicted','Filtered');

% Innovation plot for Trial 2
figure;
plot(t2, innov2(1,:), 'bo-', 'LineWidth', 1.5); hold on;
plot(t2, innov2(2,:), 'ro-', 'LineWidth', 1.5);
yline(0,'k--');
grid on;
xlabel('Time step ');
ylabel('Innovation');
title('Trial 2 Innovation');
legend('\tilde{y}_1','\tilde{y}_2');

%% Part C

% Ground truth
t = 1:5;

x1_true = t + 3;
x2_true = 2*t + 4;
v1_true = ones(1,5);
v2_true = 2*ones(1,5);

% Plot predicted velocities and corrected velocities vs. ground truth

% Trial 1
figure;
subplot(2,1,1);
plot(t, x_pred1(3,:), 'bo-', 'LineWidth', 1.5); hold on;
plot(t, x_filt1(3,:), 'ro-', 'LineWidth', 1.5);
plot(t, v1_true, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time step');
ylabel('v_1 (m/s)');
title('Trial 1 Velocity v_1');
legend('Predicted','Corrected','True');

subplot(2,1,2);
plot(t, x_pred1(4,:), 'bo-', 'LineWidth', 1.5); hold on;
plot(t, x_filt1(4,:), 'ro-', 'LineWidth', 1.5);
plot(t, v2_true, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time step');
ylabel('v_2 (m/s)');
title('Trial 1 Velocity v_2');
legend('Predicted','Corrected','True');

% Trial 2
figure;
subplot(2,1,1);
plot(t, x_pred2(3,:), 'bo-', 'LineWidth', 1.5); hold on;
plot(t, x_filt2(3,:), 'ro-', 'LineWidth', 1.5);
plot(t, v1_true, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time step');
ylabel('v_1 (m/s)');
title('Trial 2 Velocity v_1');
legend('Predicted','Corrected','True');

subplot(2,1,2);
plot(t, x_pred2(4,:), 'bo-', 'LineWidth', 1.5); hold on;
plot(t, x_filt2(4,:), 'ro-', 'LineWidth', 1.5);
plot(t, v2_true, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time step');
ylabel('v_2 (m/s)');
title('Trial 2 Velocity v_2');
legend('Predicted','Corrected','True');

% Plot estimation error

% Trial 1 corrected errors
pos_err1 = [x_filt1(1,:) - x1_true,  x_filt1(2,:) - x2_true];
vel_err1 = [x_filt1(3,:) - v1_true,  x_filt1(4,:) - v2_true];

% Trial 2 corrected errors
pos_err2 = [x_filt2(1,:) - x1_true,  x_filt2(2,:) - x2_true];
vel_err2 = [x_filt2(3,:) - v1_true,  x_filt2(4,:) - v2_true];

% Combine together
pos_err_all = [pos_err1 pos_err2];
vel_err_all = [vel_err1 vel_err2];

% Histogram of position error
figure;
histogram(pos_err_all, 5);
grid on;
xlabel('Position error (m)');
ylabel('Count');
title('Position Estimation Error Histogram');

% Histogram of velocity error
figure;
histogram(vel_err_all, 5);
grid on;
xlabel('Velocity error (m/s)');
ylabel('Count');
title('Velocity Estimation Error Histogram');

% Compute RMSE for position and velocity
rmse_pos = sqrt(mean(pos_err_all.^2));
rmse_vel = sqrt(mean(vel_err_all.^2));

disp('RMSE for position:');
disp(rmse_pos);

disp('RMSE for velocity:');
disp(rmse_vel);


%% Part E

% Different Q values
Qa = 0.01 * eye(4);
Q0 = eye(4); 
Qb = 100 * eye(4);

% True position
x1_true = t + 3;
x2_true = 2*t + 4;

% Trial 1 with Qa
N1 = size(Y1,2);
xhat = xhat0;
P = P0;
x_filt1_a = zeros(4,N1);

for k = 1:N1
    x_minus = Ad * xhat;
    P_minus = Ad * P * Ad' + Qa;

    y_tilde = Y1(:,k) - Cd * x_minus;
    S = Cd * P_minus * Cd' + R;
    K = P_minus * Cd' / S;

    xhat = x_minus + K * y_tilde;
    P = (eye(4) - K * Cd) * P_minus;

    x_filt1_a(:,k) = xhat;
end

% Trial 1 with original Q
xhat = xhat0;
P = P0;
x_filt1_0 = zeros(4,N1);

for k = 1:N1
    x_minus = Ad * xhat;
    P_minus = Ad * P * Ad' + Q0;

    y_tilde = Y1(:,k) - Cd * x_minus;
    S = Cd * P_minus * Cd' + R;
    K = P_minus * Cd' / S;

    xhat = x_minus + K * y_tilde;
    P = (eye(4) - K * Cd) * P_minus;

    x_filt1_0(:,k) = xhat;
end

% Trial 1 with Qb
xhat = xhat0;
P = P0;
x_filt1_b = zeros(4,N1);

for k = 1:N1
    x_minus = Ad * xhat;
    P_minus = Ad * P * Ad' + Qb;

    y_tilde = Y1(:,k) - Cd * x_minus;
    S = Cd * P_minus * Cd' + R;
    K = P_minus * Cd' / S;

    xhat = x_minus + K * y_tilde;
    P = (eye(4) - K * Cd) * P_minus;

    x_filt1_b(:,k) = xhat;
end

% Plot corrected position estimates for Trial 1
figure;
subplot(2,1,1);
plot(t, x_filt1_a(1,:), 'bo-', 'LineWidth', 1.5); hold on;
plot(t, x_filt1_0(1,:), 'ko-', 'LineWidth', 1.5);
plot(t, x_filt1_b(1,:), 'ro-', 'LineWidth', 1.5);
grid on;
xlabel('Time step');
ylabel('x_1 (m)');
title('Trial 1 Corrected Position x_1');
legend('Q_a = 0.01I', 'Q = I', 'Q_b = 100I');

subplot(2,1,2);
plot(t, x_filt1_a(2,:), 'bo-', 'LineWidth', 1.5); hold on;
plot(t, x_filt1_0(2,:), 'ko-', 'LineWidth', 1.5);
plot(t, x_filt1_b(2,:), 'ro-', 'LineWidth', 1.5);
grid on;
xlabel('Time step');
ylabel('x_2 (m)');
title('Trial 1 Corrected Position x_2');
legend('Q_a = 0.01I', 'Q = I', 'Q_b = 100I');

% Trial 1 position RMSE
err1_a = [x_filt1_a(1,:) - x1_true, x_filt1_a(2,:) - x2_true];
err1_0 = [x_filt1_0(1,:) - x1_true, x_filt1_0(2,:) - x2_true];
err1_b = [x_filt1_b(1,:) - x1_true, x_filt1_b(2,:) - x2_true];

rmse1_a = sqrt(mean(err1_a.^2));
rmse1_0 = sqrt(mean(err1_0.^2));
rmse1_b = sqrt(mean(err1_b.^2));

disp('Trial 1 position RMSE:');
disp(['Qa = 0.01I: ', num2str(rmse1_a)]);
disp(['Q = I: ', num2str(rmse1_0)]);
disp(['Qb = 100I: ', num2str(rmse1_b)]);

% Trial 2 with Qa
N2 = size(Y2,2);
xhat = xhat0;
P = P0;
x_filt2_a = zeros(4,N2);

for k = 1:N2
    x_minus = Ad * xhat;
    P_minus = Ad * P * Ad' + Qa;

    y_tilde = Y2(:,k) - Cd * x_minus;
    S = Cd * P_minus * Cd' + R;
    K = P_minus * Cd' / S;

    xhat = x_minus + K * y_tilde;
    P = (eye(4) - K * Cd) * P_minus;

    x_filt2_a(:,k) = xhat;
end

% Trial 2 with original Q
xhat = xhat0;
P = P0;
x_filt2_0 = zeros(4,N2);

for k = 1:N2
    x_minus = Ad * xhat;
    P_minus = Ad * P * Ad' + Q0;

    y_tilde = Y2(:,k) - Cd * x_minus;
    S = Cd * P_minus * Cd' + R;
    K = P_minus * Cd' / S;

    xhat = x_minus + K * y_tilde;
    P = (eye(4) - K * Cd) * P_minus;

    x_filt2_0(:,k) = xhat;
end

% Trial 2 with Qb
xhat = xhat0;
P = P0;
x_filt2_b = zeros(4,N2);

for k = 1:N2
    x_minus = Ad * xhat;
    P_minus = Ad * P * Ad' + Qb;

    y_tilde = Y2(:,k) - Cd * x_minus;
    S = Cd * P_minus * Cd' + R;
    K = P_minus * Cd' / S;

    xhat = x_minus + K * y_tilde;
    P = (eye(4) - K * Cd) * P_minus;

    x_filt2_b(:,k) = xhat;
end

% Trial 2 position RMSE
err2_a = [x_filt2_a(1,:) - x1_true, x_filt2_a(2,:) - x2_true];
err2_0 = [x_filt2_0(1,:) - x1_true, x_filt2_0(2,:) - x2_true];
err2_b = [x_filt2_b(1,:) - x1_true, x_filt2_b(2,:) - x2_true];

rmse2_a = sqrt(mean(err2_a.^2));
rmse2_0 = sqrt(mean(err2_0.^2));
rmse2_b = sqrt(mean(err2_b.^2));

disp('Trial 2 position RMSE:');
disp(['Qa = 0.01I: ', num2str(rmse2_a)]);
disp(['Q = I: ', num2str(rmse2_0)]);
disp(['Qb = 100I: ', num2str(rmse2_b)]);