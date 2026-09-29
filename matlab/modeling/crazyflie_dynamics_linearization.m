clear; clc; close

% Motor 1 (Front-Left) 
% Motor 2 (Front-Right)
% Motor 3 (Rear-Right)
% Motor 4 (Rear-Left)

% Mass, Gravity, Thrust Constants, Thrust-Torque, Arm length
syms m g A B k l                          

% States 
syms x y z dx dy dz phi_deg theta_deg psi_deg p_deg q_deg r_deg

% Transfer degree to radian
deg2rad = sym(pi)/180;
% deg2rad = 1; % No conversion

phi = deg2rad * phi_deg;
theta = deg2rad * theta_deg;
psi = deg2rad * psi_deg;

p = deg2rad * p_deg;
q = deg2rad * q_deg;
r = deg2rad * r_deg;

% Inputs
syms PWM1 PWM2 PWM3 PWM4

% Drone Inertia
syms J11 J12 J13 J22 J23 J33
J = [J11 J12 J13;
    J12 J22 J23;
    J13 J23 J33 ];

% Thrust
T1 = A * PWM1^2 + B * PWM1;
T2 = A * PWM2^2 + B * PWM2;
T3 = A * PWM3^2 + B * PWM3;
T4 = A * PWM4^2 + B * PWM4;
T = T1 + T2 + T3 + T4;   

% Gravity in ENU
F_gravity_ENU = [0; 0; -m * g];

% Thrust in Body Frame
F_thrust_Body = [0; 0; -T];

% Rotation Matrix from NED to Body
R_x = [1 0 0;
    0 cos(phi) sin(phi);
    0 -sin(phi) cos(phi)];

R_y = [cos(theta) 0 -sin(theta);
    0 1 0;
    sin(theta) 0 cos(theta)];

R_z = [cos(psi) sin(psi) 0;
    -sin(psi)  cos(psi) 0;
    0 0 1];

R_NED_to_Body = R_x * R_y * R_z;
R_Body_to_NED = R_NED_to_Body .';

% Rotation Matrix from NED to ENU
R_NED_to_ENU = [0 1 0;
    1 0 0;
    0 0 -1];

% Rotate thrust from Body to ENU
F_thrust_ENU = R_NED_to_ENU * R_Body_to_NED * F_thrust_Body;

% Newton's 2nd Law to calculate the acceleration
acc_ENU = (F_gravity_ENU + F_thrust_ENU) / m;

% Rotation Kinematics
W = [ 1, sin(phi)*tan(theta), cos(phi)*tan(theta);
      0, cos(phi), -sin(phi);
      0, sin(phi)*sec(theta), cos(phi)*sec(theta)];

omega = [p; -q; r];
euler_dot = W * omega;

% Conversion from radian to degree
rad2deg = 180/sym(pi);
% rad2deg = 1; % No conversion

phi_deg_dot = rad2deg * euler_dot(1);
theta_deg_dot = rad2deg * euler_dot(2);
psi_deg_dot = rad2deg * euler_dot(3);

% Calculate Torques
l_eff = l/sqrt(2);
L = l_eff * (T1 - T2 - T3 + T4);
M = l_eff * (T1 - T4 + T2 - T3);
N = k * (-T1 + T2 - T3 + T4);

tau = [L; M; N];

% Rotational Dynamics
omega_dot = J \ ( tau - cross(omega, J * omega) );

p_deg_dot = rad2deg * omega_dot(1);
q_deg_dot = -rad2deg * omega_dot(2);
r_deg_dot = rad2deg * omega_dot(3);

% Construct the System
xdot_sym = sym(zeros(12,1));
xdot_sym(1) = dx;
xdot_sym(2) = dy;
xdot_sym(3) = dz;
xdot_sym(4) = acc_ENU(1);
xdot_sym(5) = acc_ENU(2);
xdot_sym(6) = acc_ENU(3);
xdot_sym(7) = phi_deg_dot;
xdot_sym(8) = theta_deg_dot;
xdot_sym(9) = psi_deg_dot;
xdot_sym(10) = p_deg_dot;
xdot_sym(11) = q_deg_dot;
xdot_sym(12) = r_deg_dot;

% Linerization
U = [PWM1; PWM2; PWM3; PWM4];
X = [x;y;z;dx;dy;dz;phi_deg;theta_deg;psi_deg;p_deg;q_deg;r_deg];
f = xdot_sym;

A_sym = jacobian(f, X);
B_sym = jacobian(f, U);

% Numerical Values
f_air = 1;

m_val = 0.033;
g_val = 9.81;
l_val = 0.046;

A_val = 0.091492681 * f_air;
B_val = 0.067673604 * f_air;
k_val = 0.005964552 * f_air;

J_val = 1e-6 * [16.571710, 0.830806, 0.718277;
    0.830806, 16.655602, 1.800197;
    0.718277, 1.800197, 29.261652];

J11_val = J_val(1,1); J12_val = J_val(1,2); J13_val = J_val(1,3);
J22_val = J_val(2,2); J23_val = J_val(2,3); J33_val = J_val(3,3);

% Calculate PWM
T_hover = m_val * g_val / 4;
delta = B_val^2 + 4*A_val*T_hover;
PWM_star_1 = (-B_val + sqrt(delta)) / (2*A_val);
PWM_star_2 = (-B_val - sqrt(delta)) / (2*A_val);

if PWM_star_1 > 0
    PWM_trim = PWM_star_1;
   
else
    PWM_trim = PWM_star_2;
end


% Trim State
x0 = 0; y0 = 0; z0 = 0;
dx0 = 0; dy0 = 0; dz0 = 0;
phi0 = 0; theta0 = 0; psi0 = 0;
p0 = 0; q0 = 0; r0 = 0;

U0 = [PWM_trim; PWM_trim; PWM_trim; PWM_trim];


disp('PWM_trim ='); disp(PWM_trim * 65000);

% Substitution
sym_list = [m g A B k l ...
            J11 J12 J13 J22 J23 J33 ...
            x y z dx dy dz phi_deg theta_deg psi_deg p_deg q_deg r_deg ...
            PWM1 PWM2 PWM3 PWM4];

val_list = [m_val g_val A_val B_val k_val l_val ...
            J11_val J12_val J13_val J22_val J23_val J33_val ...
            x0 y0 z0 dx0 dy0 dz0 phi0 theta0 psi0 p0 q0 r0 ...
            U0(1) U0(2) U0(3) U0(4)];

% Substitutionin Linerization
A_trim_sym = subs(A_sym, sym_list, val_list);
B_trim_sym = subs(B_sym, sym_list, val_list);

A_trim = double(vpa(A_trim_sym,5));
B_trim = double(vpa(B_trim_sym,5));

disp('A_trim ='); disp(A_trim);
disp('B_trim ='); disp(B_trim);

% Check f
f_trim = double(vpa(subs(f, sym_list, val_list),16));
disp('f_trim ='); 
disp(f_trim);

% Eigenvalues
eigA = eig(A_trim);
disp('eig(A_trim) ='); 
disp(eigA); % Determine Stability

% State Space Models
C_trim = eye(12);
D_trim = zeros(12,4);
sys = ss(A_trim, B_trim, C_trim, D_trim);

% Check Controllability
n = size(A_trim,1);
controllability = ctrb(A_trim, B_trim);
r = rank(controllability);
disp('Size of Matrix A = '); 
disp(n)
disp('Rank of Controllability Matrix = '); 
disp(r)

% Actuator Dynamics
a_act = 0.9695404;
b_act = 0.0304596;

A_aug = zeros(n+4, n+4);
B_aug = zeros(n+4, 4);

% Augmented States State Space Matrix
A_aug(1:n,1:n) = A_trim;
A_aug(1:n, n+1:n+4) = B_trim;
A_aug(n+1:n+4, n+1:n+4) = a_act * eye(4);
A_aug(n+1:n+4, 1:n) = zeros(4,n);
B_aug(n+1:n+4, :) = b_act * eye(4);
C_aug = eye(n+4);
D_aug = zeros(n+4,4);

Ts = 0.01; 
sys_aug_d = ss(A_aug, B_aug, C_aug, D_aug, Ts);

disp('A_aug ='); disp(A_aug);
disp('B_aug ='); disp(B_aug);

% Simulation
t_end = 7;
dt = 0.01;
t = (0:dt:t_end)';
dPWM  = 0.02; % step size
U = dPWM * ones(length(t),4);

% No Actuator Dynamics
x0_12 = zeros(12,1);
[~,~,x_no] = lsim(sys, U, t, x0_12);

% Actuator Dynamics
sys_act = ss(A_aug, B_aug, eye(16), zeros(16,4));
x0_16 = zeros(16,1);
[~,~,x_act] = lsim(sys_act, U, t, x0_16);

% Plot the states
figure;
plot(t, x_no(:,3), t, x_act(:,3), 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Z');
legend('No actuator dynamics','With actuator dynamics');
title('Affects of actuator dynamics on Z ');

figure;
plot(t, x_no(:,6), t, x_act(:,6), 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Z dot');
legend('No actuator dynamics','With actuator dynamics');
title('Affects of actuator dynamics on dot(Z)');