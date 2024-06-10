%parametric Shrinking Horizon Model Predictive Control for a spacecraft
%manipulator with 3DoFs robotic arm
%1) a feedback linearization is used to linearize the system
%2) the paramenters to be optimised are the control gain values (i.e Kp/Kd)

%Michele Ambrosino, Ilya Kolmanovsky last rev 06/05/2024


clearvars
clc,close all

%some global variables we need to share between functions
global min_k max_k N dt umin umax

% Simulation and control parameters
T = 20;             % Total simulation time in seconds
dt = 0.1;           % Time step in seconds
N = floor(T/dt);    % Inital prediction horizon length 

% The system spacecraft + robotic arm has 12states, 
%6positions (x-y-psi-q1-q1-q3) and 6velocities
nx = 6; %DoFs Position System

x = [0; 0; 0; deg2rad(90);deg2rad(-90);deg2rad(-90);zeros(nx,1)];  % Initial Cofniguration
xf = [2.2; 0.3; deg2rad(40); deg2rad(90);deg2rad(-90);deg2rad(-90);zeros(nx,1)];  % Desired Final Configuration

% Init control gains = decision variables for the OCP
Kp_vec = [1;1;1;1;1;1]; %initial guess for the proportional gains
Kd_vec = [0.5;0.5;0.5;0.5;0.5;0.5]; %initial guess for the derivative gains
K = [Kp_vec;Kd_vec]; %initial guess vector

min_k = 0.1*ones(1,2*nx); %lim_min for the gains
max_k = 20*ones(1,2*nx); %lim_max for the gains
min_k(1) = 1; max_k(1) = 1; %we fix the Kp1 and Kd1 to a sub optimal value

%The saturation for the input vector
umin = -0.2; umax = 0.2;

%Definition of obstacles
global obs_diam obs_x obs_y rob_diam
obs_diam = 0.5; obs_x = 1.2; obs_y = -0.1; rob_diam = sqrt(2);


% To save results
X = zeros(2*nx, N); %state resul vecotr
X(:, 1) = x;
Jc = [];  %Cost Function vector
U = [];   %input vector
Kgain = []; %gain vector 


% Main simulation loop
for k = 1:floor(T/dt)
    disp("iteration: "+k+" / "+floor(T/dt))
    
    %call the optimization routine to obtain the K = (Kp,Kd) values  
    [K, Cost] = optimizePDGains(x, xf, K); %input: x:current state; xf:desired state; K:current value of K
    
    Kgain = [Kgain,;K']; %store K gains
    Jc = [Jc,Cost]; %store const function
    
    % Apply the PD controller with current gains (inside the saturation)
    u = computeControl(K, xf, x);
    U = [U;u'];
    
    % System dynamics update
    dx = FSS_dynamic_model(x,u);
    x = x + dx * dt;
    X(:, k) = x; %store the state

    %Shrink the horizon
    N = 200 - k;

    
end

% Plot results
t = 0:dt:T-dt; %build time vector
plotResults(t, X,xf,U,Jc,Kgain);

function [K, finalCost] = optimizePDGains(x, xf, K0)
    global min_k max_k N dt
    % Objective function for optimization
    objective = @(K) simulateSystem(K, x, xf, N, dt);
    % Inequality constraints
    nonlcon = @(K) systemConstraints(K, x, xf, N, dt);

    % Optimization options
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');

    % Solve the optimization problem to find new PD gains
    [K, finalCost] = fmincon(objective, K0, [], [], [], [], min_k, max_k, nonlcon, options);
end

function [c, ceq] = systemConstraints(K, x, xf, N, dt)
    global obs_diam obs_x obs_y rob_diam

    % Initialize constraint arrays
    c = [];
    ceq = [];
    
    % Simulate system for constraints verification
    for i = 1:N
        u = computeControl(K, xf, x); %compute control input
        dx = FSS_dynamic_model(x,u); %update state
        x = x + dx * dt;
        % Add obstacle avoidance constraints
        c = [c; -sqrt((x(1)-obs_x)^2+(x(2)-obs_y)^2) + (rob_diam/2 + obs_diam/2)];
        
    end
end

function J = simulateSystem(K, x, xf, N, dt)
    % Simulate system for N steps to evaluate the performance of gains K
    eSum = 0; uSum = 0;
    for i = 1:N
        u = computeControl(K, xf, x);  
        dx = FSS_dynamic_model(x,u);
        x = x + dx * dt;
        eSum = eSum + sum((xf - x).^2);
        uSum = uSum + sum(u.^2);
    end
    J = eSum * dt + uSum * dt; % Cost function: errors and control effort
end


function u = computeControl(K, xf, x)

global umin umax

    % Compute control inputs based on PD gains and current state
u = [K(1)*(xf(1) - x(1)) - K(7)*x(7);
     K(2)*(xf(2) - x(2)) - K(8)*x(8);
     K(3)*(xf(3) - x(3)) - K(9)*x(9);
     K(4)*(xf(4) - x(4)) - K(10)*x(10);
     K(5)*(xf(5) - x(5)) - K(11)*x(11);
     K(6)*(xf(6) - x(6)) - K(12)*x(12)];
     
u = max(min(u, umax), umin);
end

function dx = FSS_dynamic_model(x,u)

%Assuming that the system is after a feedback linearization
dx = [x(7); x(8);x(9);x(10);x(11);x(12); u];
end

function plotResults(t, X,xf,U,J,Kgain)

    figure(1);
    subplot(2,1, 1);
    plot(t, X(1,:), t, X(2,:),t, X(3,:),t, X(4,:),t, X(5,:),t, X(6,:));
    hold on
    plot(t,xf(1:6,1)*ones(1,size(t,2)),'--');
    legend('x', 'y','\psi','q_1','q_2','q_3','Location','best');
    xlabel('time[s]')
    title('Position');

    subplot(2, 1, 2);
    plot(t, X(7,:), t, X(8,:),t, X(9,:),t, X(10,:),t, X(11,:),t, X(12,:));
    legend('dx', 'dy','d\psi','dq_1','dq_2','dq_3','Location','best');
    xlabel('time[s]');
    title('Velocity');
    
    figure(2);
    subplot(2,1, 1);
    plot(t, U(:,1), t, U(:,2),t, U(:,3),t, U(:,4),t, U(:,5),t, U(:,6));
    legend('u1', 'u2','u3','u4','u5','u6','Location','best');
    title('Input');

    subplot(2, 1, 2);
    plot(t, J, 'k--');
    title('CostFnc');
    
    figure(3);
    subplot(2,1, 1); plot(t, Kgain(:,1:6), '--'); title('Kp');
    subplot(2,1, 2); plot(t, Kgain(:,7:end), '--'); title('Kd');
    
    %print_system_config
    
    
end
