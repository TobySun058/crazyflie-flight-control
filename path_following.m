clear; clc; close all;

%% Parameters
g = 9.81; % Gravity acceleration m/s^2
V = 30; % Speed of drone m/s
a_max = 0.4 * g; % Maximal lateral acceleration

R_min = V^2 / a_max; % Minimal allowable turning radius

R = 2 * R_min; % Orbit radius
L = 10 * R_min; % Straight line length

chi_inf = pi/2; % Maximum approach angle
k_path = 1 / R_min; % Rate of transition in line following
k_orbit = 1; % Rate of transition in orbit following

% Print Results
fprintf('R_min = %.4f m\n', R_min);
fprintf('R = %.4f m\n', R);
fprintf('L = %.4f m\n', L);

%% Path boundaries
line_min = -L/2 + R; % minimum of the straight line
line_max = L/2 - R; % maximum of the straight line

center_TR = [line_max; line_max]; % Center of top right orbit
center_TL = [line_min; line_max]; % Center of top left orbit
center_BL = [line_min; line_min]; % Center of bottom left orbit
center_BR = [line_max; line_min]; % Center of bottom right orbit

%% Vector Field

% Construct 2 coordinates
xv = linspace(-L/2 - 1.2*R, L/2 + 1.2*R, 41);
yv = linspace(-L/2 - 1.2*R, L/2 + 1.2*R, 41);
[X, Y] = meshgrid(xv, yv);

U = zeros(size(X)); % Horizaontal component
W = zeros(size(Y)); % Vertical component

chi_vehicle = 0; % Current heading angle

for i = 1:size(X,1)
    for j = 1:size(X,2)
        p = [X(i,j); Y(i,j); 0];

        commanded_course = roundedSquareGuidance(p, chi_vehicle, ...
            L, R, line_min, line_max, ...
            center_TR, center_TL, center_BL, center_BR, ...
            chi_inf, k_path, k_orbit); % Calculate commanded course angle

        U(i,j) = cos(commanded_course);
        W(i,j) = sin(commanded_course);
    end
end

% Plot vector field
figure;
quiver(X, Y, U, W, 0.6, 'LineWidth', 1);
hold on;
axis equal;
grid on;
xlabel('North (m)');
ylabel('East (m)');
title('Clockwise Path Following');

%% Plot commanded path

% Straight segments
plot([line_max, line_min], [L/2, L/2], 'k', 'LineWidth', 2); % top
plot([-L/2, -L/2], [line_max, line_min], 'k', 'LineWidth', 2); % left
plot([line_min, line_max], [-L/2, -L/2], 'k', 'LineWidth', 2); % bottom
plot([L/2, L/2], [line_min, line_max], 'k', 'LineWidth', 2); % right

% Orbit segments
theta = linspace(0, pi/2, 100);

plot(center_TR(1) + R*cos(theta), center_TR(2) + R*sin(theta), 'k', 'LineWidth', 2); % top-right
plot(center_TL(1) - R*cos(theta), center_TL(2) + R*sin(theta), 'k', 'LineWidth', 2); % top-left
plot(center_BL(1) - R*cos(theta), center_BL(2) - R*sin(theta), 'k', 'LineWidth', 2); % bottom-left
plot(center_BR(1) + R*cos(theta), center_BR(2) - R*sin(theta), 'k', 'LineWidth', 2); % bottom-right

%% Simulation
dt = 0.05; % time step
T_final = 1000; % simulation time
t = 0:dt:T_final;
N = length(t);

p_sim = zeros(3, N); % position history
chi_sim = zeros(1, N); % heading history
chi_commanded_sim = zeros(1, N); % commanded course angle
phi_commanded_sim = zeros(1, N); % commanded bank angle

p_sim(:,1) = [0; 0; 0]; % initial position
chi_sim(1) = pi/2; % initial heading (north)

% Max heading rate
chi_dot_max = a_max / V;

for k = 1:N-1
    
    % Commanded course angle
    chi_commanded_sim(k) = roundedSquareGuidance( ...
        p_sim(:,k), chi_sim(k), ...
        L, R, line_min, line_max, ...
        center_TR, center_TL, center_BL, center_BR, ...
        chi_inf, k_path, k_orbit);
    
    % wrapped heading error
    chi_error = atan2(sin(chi_commanded_sim(k) - chi_sim(k)), ...
                      cos(chi_commanded_sim(k) - chi_sim(k)));
    
    % proportional heading response
    chi_dot_commanded = chi_error;
    
    % Saturation
    chi_dot_commanded = min(max(chi_dot_commanded, -chi_dot_max), chi_dot_max);
    
    % Commanded bank angle
    phi_commanded_sim(k) = atan(V * chi_dot_commanded / g);
    
    % Update heading
    chi_sim(k+1) = chi_sim(k) + chi_dot_commanded * dt;
    
    % Update position
    p_sim(1,k+1) = p_sim(1,k) + V * cos(chi_sim(k)) * dt;
    p_sim(2,k+1) = p_sim(2,k) + V * sin(chi_sim(k)) * dt;
    p_sim(3,k+1) = 0;
end

% Fill last column
chi_commanded_sim(N) = chi_commanded_sim(N-1);
phi_commanded_sim(N) = phi_commanded_sim(N-1);

% Plot trajectory
figure;
quiver(X, Y, U, W, 0.6, 'LineWidth', 1);
hold on;
axis equal;
grid on;
xlabel('North (m)');
ylabel('East (m)');
title('Rounded-Square Path Following Simulation');

% Plot straight segments
plot([line_max, line_min], [L/2, L/2], 'k', 'LineWidth', 2); % top
plot([-L/2, -L/2], [line_max, line_min], 'k', 'LineWidth', 2); % left
plot([line_min, line_max], [-L/2, -L/2], 'k', 'LineWidth', 2); % bottom
plot([L/2, L/2], [line_min, line_max], 'k', 'LineWidth', 2); % right

% Plot orbit segments
theta = linspace(0, pi/2, 100);
plot(center_TR(1) + R*cos(theta), center_TR(2) + R*sin(theta), 'k', 'LineWidth', 2);
plot(center_TL(1) - R*cos(theta), center_TL(2) + R*sin(theta), 'k', 'LineWidth', 2);
plot(center_BL(1) - R*cos(theta), center_BL(2) - R*sin(theta), 'k', 'LineWidth', 2);
plot(center_BR(1) + R*cos(theta), center_BR(2) - R*sin(theta), 'k', 'LineWidth', 2);

% Plot trajectory
plot(p_sim(1,:), p_sim(2,:), 'r', 'LineWidth', 2);
plot(p_sim(1,1), p_sim(2,1), 'bo', 'MarkerFaceColor', 'b');

% Position time histories
figure;
plot(t, p_sim(1,:), 'LineWidth', 1.5);
hold on;
plot(t, p_sim(2,:), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Position (m)');
title('Position Time Histories');
legend('East position', 'North position');

% Heading time histories
figure;
chi_plot = atan2(sin(chi_sim), cos(chi_sim)); % Wrap
plot(t, chi_plot, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Heading Angle (rad)');
title('Heading Time History');

% Commanded bank angle time history
figure;
plot(t, rad2deg(phi_commanded_sim), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Commanded bank angle (deg)');
title('Commanded Bank Angle Time History');

%% Determine orbit following or line following Function
% p: current position
% course: current course angle
% L: length of square
% R: radius
% line_min, line_max: bounday of straight line segment
% center_TR, center_TL, center_BL, center_BR: centers of the 4 corner circles
% chi_inf, k_path, k_orbit: rate of transition
function course_commanded = roundedSquareGuidance(p, course, L, R, ...
    line_min, line_max, center_TR, center_TL, center_BL, center_BR, ...
    chi_inf, k_path, k_orbit)

% Extract the position
pn = p(1);
pe = p(2);


% First Quadrant: 
if pn >= 0 && pe >= 0
    
    % Check if point is in the corner square for the top-right orbit
    if pn >= line_max && pe >= line_max
        c = [center_TR; 0];
        rho = R;
        lambda = -1;   % Clockwise
        course_commanded = followOrbit(c, rho, lambda, p, course, k_orbit);
        return;
    end
    
    % Otherwise choose nearest straight edge: top or right
    dist_top   = abs(pe - L/2);
    dist_right = abs(pn - L/2);
    
    if dist_top <= dist_right
        % close to top segment
        r = [line_max; L/2; 0];
        q = [1; 0; 0];
        course_commanded = followStraightLine(r, q, p, course, chi_inf, k_path);
    else
        % close to right segment
        r = [L/2; line_min; 0];
        q = [0; -1; 0];
        course_commanded = followStraightLine(r, q, p, course, chi_inf, k_path);
    end
    return;
end


% Second Quadrant:
if pn < 0 && pe >= 0
    
    % Check if point is in the corner square for the top-left orbit
    if pn <= line_min && pe >= line_max
        c = [center_TL; 0];
        rho = R;
        lambda = -1;   % Clockwise
        course_commanded = followOrbit(c, rho, lambda, p, course, k_orbit);
        return;
    end
    
    % Otherwise choose nearest straight edge: top or left
    dist_top  = abs(pe - L/2);
    dist_left = abs(pn + L/2);
    
    if dist_top <= dist_left
        % top segment
        r = [line_max; L/2; 0];
        q = [1; 0; 0];
        course_commanded = followStraightLine(r, q, p, course, chi_inf, k_path);
    else
        % left segment
        r = [-L/2; line_max; 0];
        q = [0; 1; 0];
        course_commanded = followStraightLine(r, q, p, course, chi_inf, k_path);
    end
    return;
end

% Third Quadrant:
if pn < 0 && pe < 0
    
    % Check if point is in the corner square for the bottom-left orbit
    if pn <= line_min && pe <= line_min
        c = [center_BL; 0]; %
        rho = R; % radius
        lambda = -1; % clockwise
        course_commanded = followOrbit(c, rho, lambda, p, course, k_orbit);
        return;
    end
    
    % Otherwise choose nearest straight edge: bottom or left
    dist_bottom = abs(pe + L/2);
    dist_left   = abs(pn + L/2);
    
    if dist_bottom <= dist_left
        % bottom segment
        r = [line_min; -L/2; 0];
        q = [-1; 0; 0];
        course_commanded = followStraightLine(r, q, p, course, chi_inf, k_path);
    else
        % left segment
        r = [-L/2; line_max; 0];
        q = [0; 1; 0];
        course_commanded = followStraightLine(r, q, p, course, chi_inf, k_path);
    end
    return;
end


% Fourth Quadrant:
if pn >= 0 && pe < 0
    
    % Check if point is in the corner square for the bottom-right orbit
    if pn >= line_max && pe <= line_min
        c = [center_BR; 0];
        rho = R; % radius
        lambda = -1; % Clockwise
        course_commanded = followOrbit(c, rho, lambda, p, course, k_orbit);
        return;
    end
    
    % Otherwise choose nearest straight edge: bottom or right
    dist_bottom = abs(pe + L/2);
    dist_right  = abs(pn - L/2);
    
    if dist_bottom <= dist_right
        % bottom segment
        r = [line_min; -L/2; 0];
        q = [-1; 0; 0];
        course_commanded = followStraightLine(r, q, p, course, chi_inf, k_path);
    else
        % right segment
        r = [L/2; line_min; 0];
        q = [0; -1; 0];
        course_commanded = followStraightLine(r, q, p, course, chi_inf, k_path);
    end
    return;
end

end

%% Straight Line Following Function
% r: a point on the desired line
% q: direction of the line
% p: current vehicle position
% course: current course angle
% chi_inf: maximum approach angle
% k_path: rate of transition in line following

function course_commanded = followStraightLine(r, q, p, course, chi_inf, k_path)

% Extract the point on the desired line
rn = r(1);
re = r(2);

% Extract the direction of the line
qn = q(1);
qe = q(2);

% Extract current vehicle position
pn = p(1);
pe = p(2);

% Compute line direction angle
chi_q = atan2(qe, qn);

while chi_q - course < -pi
    chi_q = chi_q + 2*pi;
end

while chi_q - course > pi
    chi_q = chi_q - 2*pi;
end

% Compute cross track error
e_py = -sin(chi_q) * (pn - rn) + cos(chi_q) * (pe - re);

% Compute commanded course angle
course_commanded = chi_q - chi_inf * (2/pi) * atan(k_path * e_py);
end

%% Orbit following function
% course_commanded: commanded course angle
% c: orbit center
% rho: desired robot radius
% lambda: direction
% p: current drone position
% course: current course angle
% k_orbit: rate of transition in orbit following

function course_commanded = followOrbit(c, rho, lambda, p, course, k_orbit)
% Extract orbit center
cn = c(1);
ce = c(2);

% Extract drone position
pn = p(1);
pe = p(2);

% Distance
d = sqrt((pn - cn)^2 + (pe - ce)^2);

% Angle
varphi = atan2(pe - ce, pn - cn);

while varphi - course < -pi
    varphi = varphi + 2*pi;
end

while varphi - course > pi
    varphi = varphi - 2*pi;
end

% Compute commanded course angle
course_commanded = varphi + lambda * (pi/2 + atan(k_orbit * (d - rho) / rho));
end