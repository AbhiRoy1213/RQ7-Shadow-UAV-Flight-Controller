%% RQ-7 Shadow Baseline, Specific conditions are not exact due to information being proprietery.
%% Developed 6-DOF longitudinal/lateral autopilot in MATLAB/Simulink for a tactical UAV benchmark (RQ-7 Shadow class), evaluating stability margins and pitch/altitude tracking across aerodynamic parameter variations.
clear; clc;

%% 1. Flight Condition & Atmosphere (1500 m Cruise)
rho = 1.0581;               % Density (kg/m^3)
V0  = 40.0;                 % Trim True Airspeed (m/s)
g   = 9.81;                 % Gravity (m/s^2)
q_bar = 0.5 * rho * V0^2;   % Dynamic pressure (Pa)

%% 2. Aircraft Geometry & Mass Properties (RQ-7B Shadow)
m    = 170.0;               % Mass (kg)
S    = 2.28;                % Wing planform area (m^2)
c    = 0.54;                % Mean aerodynamic chord (m)
b    = 4.30;                % Wingspan (m)

Ixx  = 35.0;                % kg*m^2
Iyy  = 48.5;                % kg*m^2
Izz  = 75.0;                % kg*m^2
Ixz  = 2.1;                 % kg*m^2

%% 3. Non-Dimensional Aerodynamic Derivatives (Longitudinal)
% NACA 4415 based, inverted V-tail tactical configuration
CL_0 = 0.38;                % Trim lift coefficient
CD_0 = 0.045;               % Parasitic drag coefficient
CL_a = 4.80;                % Lift curve slope (/rad)
CD_a = 0.30;                % Drag due to alpha (/rad)
Cm_a = -0.65;               % Static longitudinal stability (/rad)
Cm_q = -11.5;               % Pitch damping derivative (/rad)
CL_de = 0.42;               % Lift control derivative (elevator)
Cm_de = -1.12;              % Elevator control effectiveness (/rad)

%% 4. Dimensional Longitudinal Derivatives
%Converting nondimensional derivatives into dimensional derivatives

%Forces along X (forward) and Z (downward) axis due to forward speed
%perturbations
Xu = -(2 * CD_0) * q_bar * S / (m * V0); %Speed damping in X axis
Zu = -(2 * CL_0) * q_bar * S / (m * V0); %Faster velocity increases lift
Mu = 0;  %Pitch moment

%Forces and moments cause by changes in angle of attack
Xa = -(CD_a - CL_0) * q_bar * S / m; %Increasing a slows down aircraft
Za = -(CL_a + CD_0) * q_bar * S / m; %Increasing a produces lift
Ma =  Cm_a * q_bar * S * c / Iyy; %Angular acceleration in pitch per angle of attack

%Forces and moments caused by pitch rate (q)
Xq = 0;
Zq = 0;
Mq =  Cm_q * q_bar * S * (c^2) / (2 * Iyy * V0); %Pitch damping angular acceleration per unit pitch rate

% Elevator inputs, Controls
X_de = 0;
Z_de = -CL_de * q_bar * S / m; %Vertical acceleration
M_de =  Cm_de * q_bar * S * c / Iyy; %Angular pitch acceleration

%% 5. Longitudinal State-Space Model
% x_dot = Ax + Bu
% State vector: x = [u (m/s), w (m/s), q (rad/s), theta (rad)]
% Input vector: u = delta_e (rad) (elevator deflection)
A_lon = [ Xu,        Xa/V0,      0,       -g;
          Zu*V0,     Za/V0,      V0,       0;
          Mu,        Ma/V0,      Mq,       0;
          0,         0,          1,        0 ];

B_lon = [ X_de;
          Z_de / V0;
          M_de;
          0   ];

C_lon = eye(4);
D_lon = zeros(4, 1);

sys_lon = ss(A_lon, B_lon, C_lon, D_lon, ...
    'StateName', {'u', 'w', 'q', 'theta'}, ...
    'InputName', {'delta_e'}, ...
    'OutputName', {'u', 'w', 'q', 'theta'});

%% 6. Modal Analysis
fprintf('=== RQ-7 SHADOW LONGITUDINAL DYNAMIC MODES ===\n');
[OL_wn, OL_zeta, OL_p] = damp(sys_lon);
damp(sys_lon);

%% Output: Produces a table with the 2 poles, damping ratio, frequency and time constant
% 2 complex conjugate pairs being the 2 poles, the short period with a
% higher frequency and faster settling time, and the phugoid with a lower
% frequency and longer settling time. (lambda = sigma +- jw_d)
% Magnitude (w_n, sigma): Identifies the identity of the mode (High = Short-Period, Low = Phugoid).Sign (+ or -): Identifies the stability of that mode (Negative real part = Stable, Positive real part = Unstable).


%%LONGITUDINAL CONTROL LOOPS


%% Now implementing inner control loop to increase damping to aerospace standard of around .707 for the short period 

%%Inner loop Pitch Rate Damper
%% 7. Extract Transfer Function: Elevator to Pitch Rate (q)
tf_lon = tf(sys_lon);
G_q_de = tf_lon('q', 'delta_e');

fprintf('\nOpen-Loop Transfer Function G_{q/delta_e}(s):\n');
G_q_de

%% 8. Root Locus Analysis
figure('Name', 'Pitch Rate Damper Root Locus', 'Color', 'w');
rlocus(-G_q_de); % Negative feedback convention
title('Root Locus: Elevator (\delta_e) to Pitch Rate (q)');
grid on;

% Target short-period damping line (zeta = 0.707)
sgrid(0.707, []);

%% 9. Select Gain Kq and Form Closed-Loop System
% Based on root locus inspection to place short-period poles near zeta = 0.7

%Kq = ((2*.707*OL_wn(4,:)) + (Za/V0) + Mq)/M_de;
Kq = -.1768; % Proportional rate gain (rad elevator per rad/s pitch rate)
%COME BACK TO THIS

% Form closed-loop system using feedback
% Plant input: delta_e, Plant output: q
sys_cl_inner = feedback(sys_lon, Kq, 1, 3); 
% Arguments: sys, gain K, input index (1 = delta_e), output index (3 = q)

fprintf('\n=== CLOSED-LOOP DYNAMIC MODES (WITH PITCH DAMPER Kq = %.2f) ===\n', Kq);
damp(sys_cl_inner);

%% 10. Step Response Comparison: Open-Loop vs Closed-Loop
figure('Name', 'Pitch Rate Step Response', 'Color', 'w');
opt = stepDataOptions('StepAmplitude', deg2rad(5)); % 5 deg/s pitch rate command

subplot(2,1,1);
step(sys_lon('q', 'delta_e'), 5);
title('Open-Loop Pitch Rate Response to 1 rad Elevator Step');
ylabel('q [rad/s]'); grid on;

subplot(2,1,2);
step(sys_cl_inner('q', 1), 5);
title(sprintf('Closed-Loop Pitch Rate Response (K_q = %.2f)', Kq));
ylabel('q [rad/s]'); xlabel('Time [s]'); grid on;


%%Outcome: The Short period has now been stabilized with a damping ratio of
% .658. This now eliminated pitch overshoot and set up the outer loop
% control to stabilize the phugoid mode.



%% Moving onto the outer loop control to stabilize the phugoid mode. Main point to look out for is to keep the short mode damping relatively similar while stabilizing the phugoid
% The outer loop should take around 3-5 times slower than the inner loop to
% not coincide with the inner loop. The inner loop should seem instant
% compared to the outer loop

%% 11. Pitch Attitude Loop (Outer Loop)
% Extract the SISO open plant from delta_e_cmd to theta
% Output 4 is theta, Input 1 is delta_e_cmd
G_theta = sys_cl_inner('theta', 1);

% Root locus of the attitude loop
figure('Name', 'Pitch Attitude Root Locus', 'Color', 'w');
rlocus(-G_theta); % Negative feedback convention
title('Root Locus: Elevator Command to Pitch Angle (\theta)');
grid on;

%%Output: Script to control phugoid mode. Trying not to affect the inner
%%control loop as much as possible while designing outer loop

% Gain selection
% Start with K_theta (negative because nose-up requires trailing-edge up elevator)
%When picking K_theta we should be trying to get the short mode damping
%around .7 and w_n around 3 times higher than the phugoid mode. While the
%Phugoid should have a damping around .6 and w_n 3 times less than short
%period
K_theta = -.6179; 

% Close the outer attitude loop:
% cmd -> [ K_theta ] -> (+) -> [ sys_cl_inner ] ---> theta
%                        - ^                           |
%                          +---------------------------+
sys_cl_pitch = feedback(K_theta * G_theta, 1);

fprintf('\n=== PITCH ATTITUDE CLOSED-LOOP MODES (K_theta = %.2f) ===\n', K_theta);
damp(sys_cl_pitch);

%% 12. Simulate Pitch Step Response
figure('Name', 'Pitch Angle Step Response', 'Color', 'w');
cmd_deg = 5; % 5-degree commanded pitch step
opt = stepDataOptions('StepAmplitude', deg2rad(cmd_deg));

% Plot response scaled to degrees
[theta_rad, t_out] = step(sys_cl_pitch * deg2rad(cmd_deg), 12);
plot(t_out, rad2deg(theta_rad), 'LineWidth', 1.8); hold on;
yline(cmd_deg, 'r--', 'Command (5^\circ)', 'LineWidth', 1.2);
title(sprintf('Pitch Attitude Response (K_q = %.2f, K_\\theta = %.2f)', Kq, K_theta));
xlabel('Time [s]'); ylabel('Pitch Angle \theta [deg]');
grid on;




%% 13. Altitude Kinematics (The Corrected Dimensional Math)

% 1. Extract the Constant-Airspeed Subsystem
% We isolate the states [w, q, theta] and delete [u]. 
% This acts as a perfect autothrottle, ensuring the UAV doesn't stall while climbing.
A_cs = A_lon(2:4, 2:4); 
B_cs = B_lon(2:4, 1);   
C_cs = eye(3);          
D_cs = zeros(3,1);
sys_cs = ss(A_cs, B_cs, C_cs, D_cs);

% 2. Re-close Pitch Rate Damper
% We wrap the Kq gain back around output 2 (which is now q).
sys_inner_cs = feedback(sys_cs, Kq, 1, 2);

% 3. Re-close Pitch Attitude Loop
% We wrap the K_theta gain back around output 3 (which is now theta).
sys_fb_cs = feedback(sys_inner_cs, K_theta, 1, 3);

% 4. Pre-Multiply the Input
% sys_fb_cs currently expects elevator deflection. By multiplying the input 
% by K_theta, the system now accepts theta_cmd.
sys_cl_pitch_cs = sys_fb_cs * K_theta;

% 5. Extract Kinematics: h_dot = -w + V0*theta
% Our states are [w, q, theta]. 
% We multiply w by -1, q by 0, and theta by V0 (40 m/s).
T_kinematics_cs = [-1, 0, V0];
sys_h_dot = T_kinematics_cs * sys_cl_pitch_cs;

% 6. Integrate to get altitude (h)
% We divide by 's' (Laplace for integral). No V0 multiplier here because 
% the dimensions of w and V0*theta are already in m/s!
s = tf('s');
G_h = (1 / s) * sys_h_dot; 

%% 14. Altitude PD Controller Synthesis
% Kp looks at altitude error. Kd acts as a shock absorber on the climb rate.
% For a 40 m/s UAV, these small gains command a smooth, realistic climb.
Kp_h = 0.01; 
Kd_h = 0.01; 
Ki_h = 0.0;        

C_h = pid(Kp_h, Ki_h, Kd_h);

% Close the outermost altitude loop
sys_cl_alt = feedback(C_h * G_h, 1);

fprintf('\n=== FINAL ALTITUDE HOLD MODES ===\n');
damp(sys_cl_alt);

%% 15. Simulate a 10-Meter Climb Command
figure('Name', 'Altitude Step Response', 'Color', 'w');
opt = stepDataOptions('StepAmplitude', 10); 

% Simulate out to 40 seconds to watch the full climb and leveling off
[h_out, t_out] = step(sys_cl_alt, 40, opt); 
plot(t_out, h_out, 'b', 'LineWidth', 2); hold on;
yline(10, 'r--', 'Target Altitude (10m)', 'LineWidth', 1.2);
title('10m Altitude Climb (Constant Airspeed)');
xlabel('Time [s]'); ylabel('Altitude [m]');
grid on;

%%Output: Altitude hold control. Creating loop to control and altitude
%%change and stabilize. Simulates a 10 m climb and seeing if it will level
%%off to test


%%LONGITUDINAL CONTROLS COMPLETED ABOVE


%%LATERAL CONTROLS VVVVV


%% 16. Lateral-Directional State-Space Initialization
% Dimensional aerodynamic stability derivatives for the RQ-7 class (at V0 = 40 m/s)
% Note: Using standard GNC approximations for tactical V-tail/conventional setups
Y_v = -0.12;  Y_p = 0.0;   Y_r = 0.0;    % Side-force derivatives
L_v = -15.0;  L_p = -3.5;  L_r = 1.0;    % Rolling moment derivatives
N_v = 4.0;    N_p = -0.3;  N_r = -0.6;   % Yawing moment derivatives

% Control derivatives for Aileron (da) and Rudder (dr)
L_da = 25.0;  L_dr = 1.5; 
N_da = -1.0;  N_dr = -8.0; 
Y_da = 0.0;   Y_dr = 2.0;

% Construct the A_lat Matrix: x = [beta, p, r, phi]^T
A_lat = [ Y_v/V0,     Y_p/V0,    (Y_r/V0 - 1),  g/V0;
    L_v,        L_p,       L_r,           0;
    N_v,        N_p,       N_r,           0;
    0,          1,         0,             0 ];

% Construct the B_lat Matrix: u = [delta_a, delta_r]^T
B_lat = [ Y_da/V0,    Y_dr/V0;
    L_da,       L_dr;
    N_da,       N_dr;
    0,          0 ];

C_lat = eye(4);
D_lat = zeros(4, 2);

sys_lat = ss(A_lat, B_lat, C_lat, D_lat, ...
    'StateName', {'beta', 'p', 'r', 'phi'}, ...
    'InputName', {'delta_a', 'delta_r'}, ...
    'OutputName', {'beta', 'p', 'r', 'phi'});

fprintf('\n=== LATERAL-DIRECTIONAL OPEN-LOOP MODES ===\n');
damp(sys_lat);

%% 17. Yaw Damper Synthesis (Inner Lateral Loop)
% Extract the open-loop transfer function from Rudder to Yaw Rate
% Output 3 is r, Input 2 is delta_r
G_yaw = sys_lat('r', 'delta_r');

figure('Name', 'Yaw Damper Root Locus', 'Color', 'w');
rlocus(G_yaw);
title('Root Locus: Rudder (\delta_r) to Yaw Rate (r)');
grid on;

% Target Dutch Roll damping: zeta = 0.5 to 0.7
% We apply a proportional gain to the rudder to fight the yaw rate.
% (Note: Check the root locus to see if positive or negative gain is needed 
% to pull the complex poles to the left. Usually positive for standard definitions).
K_r = -0.2; 

% Close the inner yaw loop
% Input 2 (delta_r), Output 3 (r)
sys_cl_yaw = feedback(sys_lat, K_r, 2, 3);

fprintf('\n=== LATERAL MODES (WITH YAW DAMPER K_r = %.2f) ===\n', K_r);
damp(sys_cl_yaw);

%%Output: State model showing the 3 poles from lateral control. The complex
%%conjugate pair being the dutch roll mode, the roll subsidence mode with a
%%faster frequency and the spiral mode with a slower frequency. All poles
%%should be negative showing they are dynamically stable with the spiral
%%and roll mode being critically damped

%% Roll Controller (Outer Loop)

%% 18. Roll Attitude Controller (PI Synthesis)
% Extract the open-loop transfer function from Aileron to Bank Angle.
G_phi = sys_cl_yaw('phi', 'delta_a');

% Upgrade to a PI controller to eliminate steady-state error.
% Kp provides the fast initial roll, Ki pushes through the final few degrees.
Kp_phi = .5; 
Ki_phi = 0.2; 
C_phi = pid(Kp_phi, Ki_phi);

% Close the outer roll loop with the controller in the forward path
sys_cl_roll = feedback(C_phi * G_phi, 1);

fprintf('\n=== FINAL LATERAL MODES (PI BANK HOLD) ===\n');
damp(sys_cl_roll);

%% 19. Simulate a 15-Degree Bank Angle Command
figure('Name', 'Bank Angle Step Response', 'Color', 'w');
cmd_phi_deg = 15; 
opt = stepDataOptions('StepAmplitude', cmd_phi_deg);

% Simulate the closed-loop system response for 15 seconds
[phi_out, t_out] = step(sys_cl_roll, 15, opt);

plot(t_out, phi_out, 'b', 'LineWidth', 2); hold on;
yline(cmd_phi_deg, 'r--', 'Command (15^\circ)', 'LineWidth', 1.2);
title('UAV Response to a 15^\circ Bank Angle Command (PI Control)');
xlabel('Time [s]'); ylabel('Bank Angle \phi [deg]');
grid on;


%%Output: Simulating a 15 degree bank angle to see if PI controller works
%%effectively. From simulation we see around a 3-4 second control time
%%which is a good mix of agility and speed for a UAV as opposed to a
%%passenger airline.


%%Discretizing s domain functions: Moving to real world applications these
%%former controls don't take into account the mechanical and digital lag
%%that occurs in the real world. So moving the functions from the s domain
%%to z domain tests if these controllers will work in the real world.


%% 20. Discretize Controllers for Digital Flight Hardware
% Set the microprocessor sample rate to 50 Hz (0.02 seconds per tick)
Ts = 0.02; 

fprintf('\n=== DISCRETE-TIME CONTROLLER GAINS (50 Hz) ===\n');

% 1. Discretize the Altitude PD Controller (using Tustin method)
C_h_discrete = c2d(C_h, Ts, 'tustin');
disp('Digital Altitude Controller C_h(z):');
C_h_discrete

% 2. Discretize the Roll PI Controller
C_phi_discrete = c2d(C_phi, Ts, 'tustin');
disp('Digital Roll Controller C_phi(z):');
C_phi_discrete

% 3. Quick Stability Check: Compare continuous vs discrete step response
% We will test the roll loop to ensure the digital clock speed didn't ruin our tuning.
sys_cl_roll_discrete = feedback(C_phi_discrete * c2d(G_phi, Ts, 'zoh'), 1);

figure('Name', 'Digital vs Continuous Bank Angle', 'Color', 'w');
step(sys_cl_roll, 15); hold on;
step(sys_cl_roll_discrete, 15);
legend('Perfect Continuous Math', 'Realistic 50Hz Digital Code', 'Location', 'SouthEast');
title('15^\circ Bank Angle: Continuous vs. Digital Implementation');
grid on;

%% 21. Discrete Longitudinal Stability Check (Altitude)
% Convert the continuous altitude plant to discrete time using Zero-Order Hold (ZOH)
% This simulates how the physical sensors are read in distinct 50Hz "ticks"
G_h_discrete = c2d(G_h, Ts, 'zoh');

% Close the discrete altitude loop
sys_cl_alt_discrete = feedback(C_h_discrete * G_h_discrete, 1);

% Compare continuous vs digital 10-meter climb
figure('Name', 'Digital vs Continuous Altitude', 'Color', 'w');
opt = stepDataOptions('StepAmplitude', 10); 

step(sys_cl_alt, 40, opt); hold on;
step(sys_cl_alt_discrete, 40, opt);
legend('Perfect Continuous Math', 'Realistic 50Hz Digital Code', 'Location', 'SouthEast');
title('10m Altitude Climb: Continuous vs. Digital Implementation');
ylabel('Altitude Change [m]'); xlabel('Time [s]');
grid on;