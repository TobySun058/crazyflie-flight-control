clear; clc; close all;

%% Parameters and Limit
A = 0.091492681; % thrust coefficient A
B = 0.067673604; % thrust coefficient B
k = 0.005964552; % torque coefficient
l = 0.046; 
l_eff = l / sqrt(2); % arm length 

% PWM allowable range
PWM_min = 0.0;
PWM_max = 1.0;

% Thrust allowable range
T_min = A * PWM_min^2 + B * PWM_min;
T_max = A * PWM_max^2 + B * PWM_max;

fprintf('Thrust allowable range: %f to %f\n', T_min, T_max);

%% Mixer System
mix = [ 1, 1, 1, 1;
    l_eff, -l_eff, -l_eff, l_eff;
    l_eff, l_eff, -l_eff, -l_eff;
    -k, k, -k, k ];

% Example
T_des = 0.12; % desired thrust
L_des = 0.001; % desired roll moment
M_des = 0.0001; % desired pitch moment
N_des = 0.0001; % desired yaw moment
u_des = [T_des; L_des; M_des; N_des]; 

% Desired Thrust
T_motor = mix \ u_des;

disp('Desired motor thrusts:');
disp(T_motor);

% Check feasibility
 
if ~all(T_motor >= T_min) && all(T_motor <= T_max)
    fprintf('Command is infeasible');
    alpha_vals = linspace(1, 0, 1001);
    feasible_found = false;

    for alpha = alpha_vals
        u_test = [T_des; alpha*L_des; alpha*M_des; alpha*N_des];
        T_test = mix \ u_test;
        
        if all(T_test >= T_min) && all(T_test <= T_max)
            T_motor = T_test;
            feasible_found = true;
            fprintf('Feasible solution with alpha = %f\n', alpha);
            break;
        end
    end
    
    if ~feasible_found
        error('No feasible actuator command found even after scaling moments.');
    end
end

disp('Final feasible motor thrusts:');
disp(T_motor);

%% Convert thrust to PWM
PWM = zeros(4,1);

for i = 1:4
    Ti = T_motor(i);
    
    % define  equation
    fun = @(PWM_i) A*PWM_i.^2 + B*PWM_i - Ti;
    
    % initial guess
    PWM_guess = 0.5;
    
    % solve equation
    PWM_i = fsolve(fun, PWM_guess);
    PWM(i) = PWM_i;

end

% Gurantee to a reasonbale range
PWM = min(max(PWM, PWM_min), PWM_max);

disp('Final PWM commands:');
disp(PWM * 65000);


%% Envelope calculations
T_total_max = 4*T_max;
T_total_min = 4*T_min;

L_max = 2*l_eff*(T_max - T_min);
L_min = -L_max;

M_max = 2*l_eff*(T_max - T_min);
M_min = -M_max;

N_max = 2*k*(T_max - T_min);
N_min = -N_max;

fprintf('\nAchievable envelope:\n');
fprintf('Total thrust: [%f, %f]\n', T_total_min, T_total_max);
fprintf('Roll moment: [%f, %f]\n', L_min, L_max);
fprintf('Pitch moment: [%f, %f]\n', M_min, M_max);
fprintf('Yaw moment: [%f, %f]\n', N_min, N_max);

%% Visualization
% Thrust vs PWM for one motor
PWM_grid = linspace(PWM_min, PWM_max, 500);
T_grid = A*PWM_grid.^2 + B*PWM_grid;

figure;
plot(PWM_grid, T_grid, 'LineWidth', 1.5);
grid on;
xlabel('PWM percentage');
ylabel('Motor thrust');
title('Motor thrust as a function of PWM percentage');


% Roll moment vs PWM
PWM_grid = linspace(PWM_min, PWM_max, 500);
T_grid = A*PWM_grid.^2 + B*PWM_grid;
T_min = A*PWM_min^2 + B*PWM_min;
l_eff = l/sqrt(2);

L_grid = 2*l_eff*(T_grid - T_min);

figure;
plot(PWM_grid, L_grid, 'LineWidth', 1.5);
grid on;
xlabel('PWM percentage');
ylabel('Roll moment');
title('Roll moment as a function of PWM percentage');

% Pitch moment vs PWM
PWM_grid = linspace(PWM_min, PWM_max, 500);
T_grid = A*PWM_grid.^2 + B*PWM_grid;
T_min = A*PWM_min^2 + B*PWM_min;
l_eff = l/sqrt(2);

M_grid = 2*l_eff*(T_grid - T_min);

figure;
plot(PWM_grid, M_grid, 'LineWidth', 1.5);
grid on;
xlabel('PWM percentage');
ylabel('Pitch moment');
title('Pitch moment as a function of PWM percentage');

% Yaw moment vs PWM
PWM_grid = linspace(PWM_min, PWM_max, 500);
T_grid = A*PWM_grid.^2 + B*PWM_grid;
T_min = A*PWM_min^2 + B*PWM_min;

N_grid = 2*k*(T_grid - T_min);

figure;
plot(PWM_grid, N_grid, 'LineWidth', 1.5);
grid on;
xlabel('PWM percentage');
ylabel('Yaw moment');
title('Yaw moment as a function of PWM percentage');