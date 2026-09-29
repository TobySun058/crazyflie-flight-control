%% Wrench-to-PWM actuator allocation

function [T_motor, PWM, alpha_used] = wrench_to_pwm(T_des, L_des, M_des, N_des, A, B, k, l, PWM_min, PWM_max)

% Parameters and Limit
l_eff = l / sqrt(2); % arm length 

% Thrust allowable range
T_min = A * PWM_min^2 + B * PWM_min;
T_max = A * PWM_max^2 + B * PWM_max;

% Total thrust allowable range
T_total_min = 4 * T_min;
T_total_max = 4 * T_max;

% Mixer System
mix = [1, 1, 1, 1;
    l_eff, -l_eff, -l_eff, l_eff;
    l_eff,  l_eff, -l_eff, -l_eff;
    -k, k, -k, k ];

% Handle infeasible total thrust first
if T_des < T_total_min || T_des > T_total_max
    fprintf('Desired total thrust is infeasible. Clipping total thrust.\n');
    T_des = min(max(T_des, T_total_min), T_total_max);
end

% Desired thrust and moments
u_des = [T_des; L_des; M_des; N_des];

% Desired thrust for each motor
T_motor = mix \ u_des;

alpha_used = 1.0;

% Check feasibility for L, M, N
if ~(all(T_motor >= T_min) && all(T_motor <= T_max))
    fprintf('Desired wrench is infeasible. Scaling L, M, and N.\n');

    % Find largest alpha to satisfy thrust limit
    alpha_vals = linspace(1, 0, 1001);
    feasible_found = false;

    for alpha = alpha_vals
        u_test = [T_des; alpha*L_des; alpha*M_des; alpha*N_des];
        T_test = mix \ u_test;

        if all(T_test >= T_min) && all(T_test <= T_max)
            T_motor = T_test;
            alpha_used = alpha;
            feasible_found = true;
            fprintf('Feasible solution with alpha = %f\n', alpha);
            break;
        end
    end

    if ~feasible_found
        error('No feasible actuator command found after scaling moments.');
    end
end

% Convert thrust to PWM
PWM = zeros(4,1);

for i = 1:4
    Ti = T_motor(i);

    % guarantee non-negative thrust
    Ti = max(Ti, 0);

    % solve A*PWM^2 + B*PWM - Ti = 0
    disc = B^2 + 4*A*Ti;
    PWM_i = (-B + sqrt(disc)) / (2*A);

    PWM(i) = PWM_i;
end

% Guarantee to a reasonable range for PWM
PWM = min(max(PWM, PWM_min), PWM_max);

end