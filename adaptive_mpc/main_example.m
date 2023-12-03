% Tube-based MPC
%
% Requires MPT3


%m = 1575;
%Iz = 2875;
%lf = 1.2;
%lr = 1.6;
%Cf = 19000;
%Cr = 33000;
%A = [-(2*Cf+2*Cr)/m/Vx, -Vx-(2*Cf*lf-2*Cr*lr)/m/Vx, -(2*Cf*lf-2*Cr*lr)/Iz/Vx, -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx];
%B = [2*Cf/m, 2*Cf*lf/Iz]';
%C = eye(2);
%G = ss(A,B,C,0);
%Vx = 15;

clc;
clear all;
close all;

% System dynamics
A = [1 1; 0 1]; B = [0 1]';

x0 = [4 -1]';

% Optimization problem parameters
N = 3;
Q = eye(2);
R = 0.01;
P = dare(A,B,Q,R);

% state constraint Ax * x <= bx
Ax = [eye(2); -eye(2)];
bx = 5 * ones(4,1);

% input constraint Au * u <= bu
Au = [1; -1]; 
bu = 0.5 * ones(2,1);


X = Polyhedron(Ax, bx);
U = Polyhedron(Au, bu);


%% Control invariant set

% System dynamics

figure(2)
hold on;
plot(X,'color',[0.7 0.7 0.7]);

Omega = X;
H_om = Ax; 
K_om = bx;

for i=1:100
    % Precursor of X
    Pre_XU = Polyhedron('A', [H_om * A, H_om * B ; zeros(2,2), Au], 'b', [K_om; bu]);
    Pre_X = Pre_XU.projection([1 2]);
    disp(Pre_X)

    H_pre = Pre_X.H(:,1:2);
    K_pre = Pre_X.H(:,3);

    Omega_next = Polyhedron('A', [H_pre; H_om], 'b', [K_pre; K_om]);
%     plot(Omega_next, 'color', [0.5 0.5 0.5])
    
    if Omega_next == Omega
        display('converged')
        break
    end
    
    Omega = Omega_next;
    H_om = Omega.H(:,1:2);
    K_om = Omega.H(:,3);
end

Nbar = i;
Cinf = Omega;
plot(Cinf, 'color', 'm')

Af = Cinf.H(:,1:2);
bf = Cinf.H(:,3);

%% Receding horizon control
k_max = 15;
x = zeros(size(x0,1), k_max+1);
x_open = zeros(size(x0,1), N+1, k_max);
x(:,1) = x0;

for k=1:k_max
    % Solve the finite horizon optimal control problem
    if k == 1
        [xstar, ustar, Jstar, exitflag ] = finite_control( P, Q, R, A, B, Ax, bx, Au, bu, Af, bf, N, x0)
        if exitflag ~= 1
            error('The problem is infeasible with the initial state!')
        end
    else
        [xstar, ustar, Jstar, exitflag ] = finite_control( P, Q, R, A, B, Ax, bx, Au, bu, Af, bf, N, x(:,k))
    end
    
    if exitflag ~= 1
        mssg = ['Infeasible at time step ', num2str(k)];
        display(mssg)
        break
    end
    
    x_open(:,:,k) = xstar;
    x(:,k+1) = A*x(:,k) + B*ustar(1);
    
    figure(2)
    grid on; hold on;
    title('without disturbance')
%     plot(X, 'wire', true)
    plot(x(1,1:k+1), x(2,1:k+1), '-o', 'LineWidth', 2, 'Color', 'b')
    plot(x_open(1,:,k), x_open(2,:,k), 'or--', 'LineWidth', 1.5)
    xlabel('x_1');
    ylabel('x_2'); 
    set(gca, 'fontsize', 12)
end

%% Now, the actual system is subject to additive disturbance
% Tube MPC


K = [-1 -1];
abs(eig(A+B*K))

% disturbance [-0.1, 0.1]
P_w = Polyhedron([eye(2); -eye(2)], 0.1*ones(4,1));
S = P_w;
for i=1:N-1
    S = (A + B*K) * S + P_w;
end

plot(S)

figure(1)
subplot(121)
hold on; grid on;
plot(X, 'wire', true)
plot(X-S)
title('tightened state constraint')

subplot(122)
hold on; grid on;
plot(Cinf, 'wire', true)
plot(Cinf-S)
title('tightened terminal constraint')


P_x = X -S;
P_xf = Cinf - S;
P_u = U - K*S;
p_13gh =K*S;
disp(K)
disp(S.A)
disp(S.b)
disp('A=')
disp(p_13gh.A)
disp('b=')
disp(p_13gh.b)

k_max = 50;
x = zeros(size(x0,1), k_max+1);
xbar = zeros(size(x0,1), k_max+1);
x_open = zeros(size(x0,1), N+1, k_max);
x(:,1) = x0;
xbar(:,1) = x0;

% actual disturbance
load('w.mat')
% w = 0.1 - 0.2 * rand(2,k_max); % w uniform distributed [-0.1, 0.1] x [-0.1, 0.1]


for k=1:k_max
    % Solve the finite horizon optimal control problem
    if k == 1
        [xstar, ustar, Jstar, exitflag ] = finite_control( P, Q, R, A, B, P_x.A, P_x.b, P_u.A, P_u.b, P_xf.A, P_xf.b, N, x0)
        if exitflag ~= 1
            error('The problem is infeasible with the initial state!')
        end
    else
        [xstar, ustar, Jstar, exitflag ] = finite_control( P, Q, R, A, B, P_x.A, P_x.b, P_u.A, P_u.b, P_xf.A, P_xf.b, N, x(:,k))
    end
    
    if exitflag ~= 1
        mssg = ['Infeasible at time step ', num2str(k)];
        display(mssg)
        break
    end
   
    ubar(:,k) = ustar(1);
    u(:,k) = K * (x(:,k) - xbar(:,k)) + ubar(k);
    x(:,k+1) = A * x(:,k) + B* u(:,k) + w(:,k); 
    xbar(:,k+1) = A * xbar(:,k) + B * ubar(1);
    
    figure(4)
    grid on; hold on;
    plot(X,'color',[0.7 0.7 0.7]);
    plot(x(1,1:k+1), x(2,1:k+1), '-o', 'LineWidth', 2, 'Color', 'b')
    xlabel('x_1');
    ylabel('x_2'); 
    set(gca, 'fontsize', 12)
    title('tube MPC with fixed x0')
end

%% Tube MPC - modified version for recursive feasibility

k_max = 20;
x = zeros(size(x0,1), k_max+1);
xbar = zeros(size(x0,1), k_max+1);
x_open = zeros(size(x0,1), N+1, k_max);
x(:,1) = x0;


for k=1:k_max
    % Solve the finite horizon optimal control problem
    [xstar, ustar, Jstar, exitflag ] = finite_control_modified( P, Q, R, A, B, P_x.A, P_x.b, P_u.A, P_u.b, P_xf.A, P_xf.b, N, x(:,k), S)
    if exitflag ~= 1
        mssg = ['Infeasible at time step ', num2str(k)];
        display(mssg)
        break
    end
    
    ubar(:,k) = ustar(1);
    xbar(:,k) = xstar(:,1);
    u(:,k) = K * (x(:,k) - xbar(:,k)) + ubar(k);
    x(:,k+1) = A * x(:,k) + B* u(:,k) + w(:,k); 
    xbar(:,k+1) = A * xbar(:,k) + B * ubar(:,k);
    
    figure(5)
    grid on; hold on;
    plot(X,'color',[0.7 0.7 0.7]);
    plot(x(1,1:k+1), x(2,1:k+1), '-o', 'LineWidth', 2, 'Color', 'b')
    xlabel('x_1');
    ylabel('x_2'); 
    set(gca, 'fontsize', 12)
    title('improved tube MPC')
    
end

figure(6)
grid on; hold on;
plot(X,'color',[0.7 0.7 0.7]);
plot_xbar = plot(xbar(1,:), xbar(2,:), '-o', 'LineWidth', 2, 'Color', 'r');
for k=1:k_max
    plot(xbar(:,k)+S, 'wire', true)
end
plot_x = plot(x(1,:), x(2,:), '-o', 'LineWidth', 2, 'Color', 'b');
legend([plot_xbar, plot_x], {'nominal', 'actual'})
xlabel('x_1');
ylabel('x_2'); 
set(gca, 'fontsize', 12)

e = x - xbar;

figure(7)
hold on; grid on; 
S.plot('wire', true)
plot(e(1,:), e(2,:), '-o', 'LineWidth', 2, 'Color', 'b')
