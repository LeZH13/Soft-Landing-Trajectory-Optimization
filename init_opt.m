clear all; close all; clc;
% run this first
%% Paper numerical examples

m_dry = 2000  ;       % kg
m_fuel= 300 ;      
m_wet = m_dry+m_fuel ; 

g0    = 9.80665; 

T_max = 24000   ;        % max thrust magnitude
throt = [0.2,0.8]  ;     % throttle level
Isp   = 203.94      ;    % fuel efficiency
alpha = 1/(Isp*g0)   ;   % fuel consumption alpha

V_max = 90        ;      % velocity max

r1    = throt(1)*T_max ; % lower thrust bound
r2    = throt(2)*T_max;  % upper thrust bound

g = [-3.71;0;0];
omega = [2.53*1e-5; 0; 6.62*1e-5];
nh = [1;0;0];

case_n = 3; % case used
switch case_n
    case 1 % 90 deg constrained
        p0 = [2400,450,-330]';
        v0 = [-10,-40,10]'; 
        y_gs  = deg2rad(60);  % glide slope angle
        theta  = deg2rad(90); % thrust pointing constraint
        t_f = 47; % in second
    case 2 % 45 deg constrained
        p0 = [2400,450,-330]';
        v0 = [-10,-40,10]'; 
        y_gs  = deg2rad(30);  % glide slope angle
        theta  = deg2rad(45);  % thrust pointing constraint
        t_f = 52.5; % in second
    case 3 % second example in paper
        p0 = [2400,3400,0]';
        v0 = [-40,-45,0]';
        y_gs  = deg2rad(30); 
        theta  = deg2rad(120); 
        t_f = 60;
end

% goal
p_ref = [0;0;0];
v_final = [0;0;0];

% opt time
dt = .5;
N = t_f/dt;

% start 
TrajectoryOptimization
% plot
plots

