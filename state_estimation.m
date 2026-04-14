clc; clear; close all;

% Load Satellite Data
T = readtable('Sats.txt');

sat_x = T{:,2}; % Position x
sat_y = T{:,3}; % Position y
sat_vx = T{:,4}; % Velocity x
sat_vy = T{:,5}; % Velocity y
rho_meas = T{:,6}; % Measured rho
rhodot_meas = T{:,7}; % Measured rho dot
nSat = length(sat_x); % Number of Satellites

% Settings
nIter = 50; % Iterations
sigma_r = 0.5; % Range sd
sigma_rd = 0.5; % Range rate sd
R_single = diag([sigma_r^2, sigma_rd^2]); % Covariance matrix

% Symbolic Jacobian
syms x y xdot ydot xs ys vxs vys real

dx_sym = xs - x;
dy_sym = ys - y;
dvx_sym = vxs - xdot;
dvy_sym = vys - ydot;

r_sym = sqrt(dx_sym^2 + dy_sym^2);
rhodot_sym = (dx_sym*dvx_sym + dy_sym*dvy_sym) / r_sym;

h_sym = [r_sym;
         rhodot_sym];

H_sym = jacobian(h_sym, [x y xdot ydot]);

% Initial estimate
xbar = [0; 0; 0; 0];

% Initial prior covariance
M = diag([1^2 1^2 1^2 1^2]);

% Store estimate history
x_hist = zeros(4, nIter+1);
x_hist(:,1) = xbar;

% Algorithm
for iter = 1:nIter
    
    % Use all satellites first for 3 iter, then remove west satellites
    if iter <= 3
        trusted = true(nSat,1);
    else
        trusted = sat_x >= xbar(1);
    end
    
    trusted_idx = find(trusted);
    nTrusted = length(trusted_idx);
    
    H = zeros(2*nTrusted, 4); % Jacobian matrix
    dz = zeros(2*nTrusted, 1); % Residual vector
    R = zeros(2*nTrusted); % Measurement covariance
    
    row = 1; % Initialize index
    
    for k = 1:nTrusted
        i = trusted_idx(k);
        
        % Relative position
        dx = sat_x(i) - xbar(1);
        dy = sat_y(i) - xbar(2);

        % Relative velocity
        dvx = sat_vx(i) - xbar(3);
        dvy = sat_vy(i) - xbar(4);
        
        % Predicted measurements
        r = sqrt(dx^2 + dy^2);
        rhodot = (dx*dvx + dy*dvy) / r;
        zbar_i = [r; rhodot];
        
        % Residual
        dz(row:row+1) = [rho_meas(i); rhodot_meas(i)] - zbar_i;
        
        % Jacobian
        H_i = double(subs(H_sym, ...
            [x, y, xdot, ydot, xs, ys, vxs, vys], ...
            [xbar(1), xbar(2), xbar(3), xbar(4), ...
             sat_x(i), sat_y(i), sat_vx(i), sat_vy(i)]));
        H(row:row+1,:) = H_i;

        % Covariance matrix
        R(row:row+1, row:row+1) = R_single;
        
        % Update index
        row = row + 2;
    end
    
    % Posterior covariance
    P = inv(inv(M) + H' * inv(R) * H);
    
    % Gain matrix
    K = P * H' * inv(R);
    
    % Update estimate
    xhat = xbar + K * dz;
    
    % Update for next iteration
    xbar = xhat;
    M = P;
    
    % Store estimate
    x_hist(:,iter+1) = xbar;
end

% Final Covariance
P_final = M;
P_pos = P_final(1:2,1:2); % Position covariance
P_vel = P_final(3:4,3:4); % Velocity covariance

% Display Final Estimate
disp('Final Estimated State [x; y; xdot; ydot]:');
disp(xbar);

% Iteration Axis
iter_axis = 0:nIter;

% Plot Position vs Iteration
figure;
plot(iter_axis, x_hist(1,:), 'LineWidth', 1.5); hold on;
plot(iter_axis, x_hist(2,:), 'LineWidth', 1.5);
grid on;
xlabel('Iteration');
ylabel('Position Estimate');
title('Estimated Position vs Iteration');
legend('x','y');

% Plot Velocity vs Iteration
figure;
plot(iter_axis, x_hist(3,:), 'LineWidth', 1.5); hold on;
plot(iter_axis, x_hist(4,:), 'LineWidth', 1.5);
grid on;
xlabel('Iteration');
ylabel('Velocity Estimate');
title('Estimated Velocity vs Iteration');
legend('xdot','ydot');

% Plot Final Posterior Density of Position
mu_pos = xbar(1:2);
plotDensity(mu_pos, P_pos, ...
    'Final Conditional Posterior Density of Position', 'x', 'y');

% Plot Final Posterior Density of Velocity
mu_vel = xbar(3:4);
plotDensity(mu_vel, P_vel, ...
    'Final Conditional Posterior Density of Velocity', 'xdot', 'ydot');

% Plot density function
function plotDensity(mu, P, plotTitle, label1, label2)

    % Grid based on
    s1 = sqrt(P(1,1));
    s2 = sqrt(P(2,2));
     
    x1 = linspace(mu(1)-4*s1, mu(1)+4*s1, 100);
    x2 = linspace(mu(2)-4*s2, mu(2)+4*s2, 100);
    [X1, X2] = meshgrid(x1, x2);
    X = [X1(:) X2(:)];
    
    % Evaluate bivariate normal pdf
    Z = mvnpdf(X, mu', P);
    Z = reshape(Z, length(x2), length(x1));  
    
    % Plot
    figure;
    contour(x1, x2, Z, 12, 'LineWidth', 1.2);
    hold on;
    plot(mu(1), mu(2), 'r.', 'MarkerSize', 20);
    grid on;
    xlabel(label1);
    ylabel(label2);
    title(plotTitle);

end