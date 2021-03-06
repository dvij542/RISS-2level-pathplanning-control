addpath('../src/')
addpath('../src/utils/')

% fix random seed
rng(0);

% make your own discrete linear system with disturbance
A = [1 1; 0 1];
B = [0.5; 1]; 
Q = diag([10, 10]);
R = 0.01;

W_vertex = [0.1, 0.1; 0.1, -0.1; -0.1, -0.1; -0.1, 0.1]; % construct a convex set of disturbance (2dim here)
W = Polyhedron(W_vertex);


% constraints on state Xc and input Uc
Xc_vertex = [20, -5; 20 5; -20 5; -20 -5];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);
deltaT = 0.3;
% construct disturbance Linear system
disturbance_system = DisturbanceLinearSystem_delay(A, B, Q, R, W, Uc, deltaT);

% create a tube_mpc simulater
% if N_horizon is too small, the path will never reach inside the robust MPI-set X_mpi_robust in time step N_horizon, then the problem becomes infeasible. 
N_horizon = 7;
s=4;
mpc = TubeModelPredictiveControl(disturbance_system, Xc, Uc, N_horizon);

% The robust MPC guidances the path inside the robust MPI-set so that the path will reach the robust MPI-set in N_horizon. 
x = [-5; -2];
savedir_name = './results/';
mkdir(savedir_name);
x_nom = zeros(2,N_horizon);
u_nom = zeros(1,N_horizon);
x_nom(:,1:1) = x;
for i=2:N_horizon+1
    x_nom(:,i:i) = disturbance_system.A*x_nom(:,i-1:i-1);
end
times = zeros(1,7);
trajectory1 = zeros(2,100);
k = 1;
for i = 1:6
    disp(x_nom);
    x_curr = x_nom(:,s+1) + disturbance_system.Ak^s*(x-x_nom(:,1));
    if i>1 
        mpc.show_prediction(x);
        pause(1);
        filename = strcat(savedir_name, 'tmpc_seq', number2string(k), '.png');
        saveas(gcf, char(filename)); % removing this line makes the code much faster
    end
    disp(x);
    for j=1:s
        trajectory1(:,k:k) = x;
        k=k+1;
        x = disturbance_system.propagate(x, u_nom(:,j) + disturbance_system.K*(x-x_nom(:,j)));
        disp(x); % additive disturbance is considered inside the method
        if i>1 
            mpc.show_prediction(x);
            pause(1);
            filename = strcat(savedir_name, 'tmpc_seq', number2string(k), '.png');
            saveas(gcf, char(filename)); % removing this line makes the code much faster
        end
    end
    trajectory1(:,k:k) = x;
    k=k+1;
        
    xdash = disturbance_system.propagate(x, u_nom(:,s+1) + disturbance_system.K*(x-x_nom(:,s+1)));
    val = deltaT*(2*rand()-1);
    times(1,i) = 4 + val;
    disp(val);
    if val>1 
        x = x + (xdash-x)*val;
    end
    trajectory1(:,k:k) = x;
    k=k+1;
        
    [u_nom,x_nom] = mpc.solve_delay(x_curr,s);
%     filename = strcat(savedir_name, 'tmpc_seq', number2string(i), '.png');
    saveas(gcf, char(filename)); % removing this line makes the code much faster
    clf;
end

trajectory1(:,k:k) = x;
%k=k+1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% fix random seed
rng(0);

% make your own discrete linear system with disturbance
A = [1 1; 0 1];
B = [0.5; 1]; 
Q = diag([10, 10]);
R = 0.01;

W_vertex = [0.1, 0.1; 0.1, -0.1; -0.1, -0.1; -0.1, 0.1]; % construct a convex set of disturbance (2dim here)
W = Polyhedron(W_vertex);


% constraints on state Xc and input Uc
Xc_vertex = [20, -5; 20 5; -20 5; -20 -5];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);
deltaT = 0;
% construct disturbance Linear system
disturbance_system = DisturbanceLinearSystem_delay(A, B, Q, R, W, Uc, deltaT);

% create a tube_mpc simulater
% if N_horizon is too small, the path will never reach inside the robust MPI-set X_mpi_robust in time step N_horizon, then the problem becomes infeasible. 
N_horizon = 7;
s=6;
mpc = TubeModelPredictiveControl(disturbance_system, Xc, Uc, N_horizon);

% The robust MPC guidances the path inside the robust MPI-set so that the path will reach the robust MPI-set in N_horizon. 
x = [-5; -2];
savedir_name = './results/';
mkdir(savedir_name);
x_nom = zeros(2,N_horizon);
u_nom = zeros(1,N_horizon);
x_nom(:,1:1) = x;
for i=2:N_horizon+1
    x_nom(:,i:i) = disturbance_system.A*x_nom(:,i-1:i-1);
end

trajectory2 = zeros(2,100);
k=1;
for i = 1:8
    disp(x_nom);
    x_curr = x_nom(:,s+1) + disturbance_system.Ak^s*(x-x_nom(:,1));
    if i>1 
%         mpc.show_prediction(x);
%         pause(1);
    end
    disp(x);
    for j=1:s
        trajectory2(:,k:k) = x;
        k=k+1;
        x = disturbance_system.propagate(x, u_nom(:,j) + disturbance_system.K*(x-x_nom(:,j)));
        disp(x); % additive disturbance is considered inside the method
        if i>1 
%             mpc.show_prediction(x);
%             pause(1);
        end
    end
    trajectory2(:,k:k) = x;
    k=k+1;
    [u_nom,x_nom] = mpc.solve_delay(x_curr,s);
%     filename = strcat(savedir_name, 'tmpc_seq', number2string(i), '.png');
%     saveas(gcf, char(filename)); % removing this line makes the code much faster
    clf;
end

% plot(trajectory1(1,1:29),trajectory1(2,1:29),trajectory2(1,1:29),trajectory2(2,1:29));%,trajectory(2,1:10),trajectory(3,1:10),trajectory2(2,1:29),trajectory2(3,1:29));%,trajectory(1,1:2889),trajectory(3,1:2889));
% title('Difference in path tracked for with delay consideration as constant vs variable with bounded uncertainity');
% legend({'With variable time delay T=4 N=7 deltaT=0.3','With constant time delay T=6 N=7'},'Location','northeast')
% saveas(gcf,'results/difference.png')
plot(times(1,1:6));
title('Computation times');
axis([0 6 0 6]);
saveas(gcf,'results/comp_times.png')