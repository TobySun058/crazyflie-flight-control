function [T_motor, PWM, alpha_used] = mixer_pwm_command(T_des, L_des, M_des, N_des, A, B, k, l, PWM_min, PWM_max)

    l_eff = l / sqrt(2);

    % thrust limits from PWM limits
    T_min = A * PWM_min^2 + B * PWM_min;
    T_max = A * PWM_max^2 + B * PWM_max;

    % mixer matrix
    mix = [ 1,      1,      1,      1;
            l_eff, -l_eff, -l_eff,  l_eff;
            l_eff,  l_eff, -l_eff, -l_eff;
           -k,      k,     -k,      k ];

    u_des = [T_des; L_des; M_des; N_des];
    T_motor = mix \ u_des;

    alpha_used = 1.0;

    % feasibility correction by scaling moments only
    if ~(all(T_motor >= T_min) && all(T_motor <= T_max))
        alpha_vals = linspace(1, 0, 1001);
        feasible_found = false;

        for alpha = alpha_vals
            u_test = [T_des; alpha*L_des; alpha*M_des; alpha*N_des];
            T_test = mix \ u_test;

            if all(T_test >= T_min) && all(T_test <= T_max)
                T_motor = T_test;
                alpha_used = alpha;
                feasible_found = true;
                break;
            end
        end

        if ~feasible_found
            error('No feasible actuator command found even after scaling moments.');
        end
    end

    % thrust -> PWM
    PWM = zeros(4,1);

    for i = 1:4
        Ti = T_motor(i);

        % clamp negative thrust to zero
        Ti = max(Ti, 0);

        % solve A*PWM^2 + B*PWM - Ti = 0
        disc = B^2 + 4*A*Ti;
        PWM_i = (-B + sqrt(disc)) / (2*A);

        % clamp to allowable range
        PWM(i) = min(max(PWM_i, PWM_min), PWM_max);
    end
end