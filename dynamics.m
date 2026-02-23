clear; clc; close

% Motor 1 (Front-Left) 
% Motor 2 (Front-Right)
% Motor 3 (Rear-Right)
% Motor 4 (Rear-Left)

% Mass, Gravity, Thrust Constants, Thrust-Torque, Arm length
syms m g A B k l                          

% States 
syms x y z dx dy dz phi theta psi p q r

% Inputs
syms PWM1 PWM2 PWM3 PWM4

% Drone Inertia
syms J11 J12 J13 J22 J23 J33
J = [J11 J12 J13;
    J12 J22 J23;
    J13 J23 J33 ];

% State Vector
X = [x; y; z; dx; dy; dz; phi; theta; psi; p; q; r];

% Thrust
T1 = A * PWM1^2 + B * PWM1;
T2 = A * PWM2^2 + B * PWM2;
T3 = A * PWM3^2 + B * PWM3;
T4 = A * PWM4^2 + B * PWM4;
T = T1 + T2 + T3 + T4;   

% Gravity in ENU
F_gravity_ENU = [0; 0; -m * g];

% Thrust in BODY
F_thrust_Body = [0; 0; T];

% Rotation Matrix
R_roll = [ 1, 0, 0;
    0, cos(phi), sin(phi);
    0, -sin(phi), cos(phi)];

R_pitch = [ cos(theta), 0, -sin(theta);
       0, 1, 0;
       sin(theta), 0, cos(theta)];

R_yaw = [ cos(psi), sin(psi), 0;
      -sin(psi), cos(psi), 0;
       0, 0, 1];

R_NED_to_Body = R_roll * R_pitch * R_yaw; % NED to Body
R_Body_to_NED = R_NED_to_Body.'; % Body to NED

R_NED_to_ENU = [0 1 0; 1 0 0; 0 0 -1]; % NED to ENU
R_ENU_to_NED = R_NED_to_ENU.'; % ENU to NED

R_Body_to_ENU = R_NED_to_ENU * R_Body_to_NED; % Body to ENU     

% Rotate thrust from Body to ENU
F_thrust_ENU = R_Body_to_ENU * F_thrust_Body;

% Newton's 2nd Law to calculate the acceleration
acc_ENU = (F_gravity_ENU + F_thrust_ENU) / m;

% Rotation Kinematics
W = [ 1, sin(phi)*tan(theta), cos(phi)*tan(theta);
      0, cos(phi), -sin(phi);
      0, sin(phi)*sec(theta), cos(phi)*sec(theta)];

omega = [p; -q; r];
euler_dot = W * omega;

phi_dot = euler_dot(1);
theta_dot = euler_dot(2);
psi_dot = euler_dot(3);

% Calculate Torques
L = l * (T2 - T4);
M = l * (T3 - T1);
N = k * (-T1 + T2 - T3 + T4);
tau = [L; M; N];

% Rotational Dynamics
omega_dot = J \ ( tau - cross(omega, J * omega) );

p_dot = omega_dot(1);
q_dot = -omega_dot(2);
r_dot = omega_dot(3);

% Construct the System
xdot_sym = sym(zeros(12,1));
xdot_sym(1)  = dx;
xdot_sym(2)  = dy;
xdot_sym(3)  = dz;
xdot_sym(4)  = acc_ENU(1);
xdot_sym(5)  = acc_ENU(2);
xdot_sym(6)  = acc_ENU(3);
xdot_sym(7)  = phi_dot;
xdot_sym(8)  = theta_dot;
xdot_sym(9)  = psi_dot;
xdot_sym(10) = p_dot;
xdot_sym(11) = q_dot;
xdot_sym(12) = r_dot;

xdot_sym = simplify(xdot_sym);
disp('xdot_sym = ');
disp(xdot_sym)