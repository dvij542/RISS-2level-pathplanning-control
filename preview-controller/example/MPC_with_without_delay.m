addpath('../src/')
addpath('../src/utils/')


%the usage is mostly same as tubeMPC
delay = 2;
A = [1 1; 0 1];
B = [0.5; 1];
Q = diag([1, 1]);
R = 0.01;
mysys = LinearSystem(A, B, Q, R);
Xc_vertex = [20, 2; 20, -2; -20, -2; -20, 2];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);
N_horizon = 2;
mpc = ModelPredictiveControl(mysys, Xc, Uc, N_horizon);
savedir_name = 'results';

x_curr = [-5; -2];
buffer = zeros(1,delay);
cmd = 0;
trajectory3 = zeros(3,30);
for i = 1:29
    trajectory3(1,i) = i;
    trajectory3(2,i) = x_curr(1);
    trajectory3(3,i) = x_curr(2);
    x_new = x_curr;
    for j = 1:delay
        x_new = mysys.propagate(x_new,buffer(delay+1-j));
    end
    %disp(x_new)
    u_next = mpc.solve(x_new);
    [buffer, cmd] = push(buffer,u_next);
    x_curr = mysys.propagate(x_curr, cmd); % + add some noise here
end

x_curr = [-5; -2];
buffer = zeros(1,delay);
cmd = 0;
trajectory2 = zeros(3,30);
for i = 1:29
    trajectory(1,i) = i;
    trajectory2(2,i) = x_curr(1);
    trajectory2(3,i) = x_curr(2);
    x_new = x_curr;
    disp(x_new)
    u_next = mpc.solve(x_new);
    cmd = u_next;
    x_curr = mysys.propagate(x_curr, cmd); % + add some noise here
    
end

Xc_vertex = [25, 25; 25, -25; -25, -25; -25, 25];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);
mpc = ModelPredictiveControl(mysys, Xc, Uc, N_horizon);
x_curr = [-5; -2];
buffer = zeros(1,delay);

cmd = 0;
trajectory = zeros(3,30);
for i = 1:29
    trajectory(1,i) = i;
    trajectory(2,i) = x_curr(1);
    trajectory(3,i) = x_curr(2);
    x_new = x_curr;
    disp(i)
    u_next = mpc.solve(x_new);
    [buffer, cmd] = push(buffer,u_next);
    x_curr = mysys.propagate(x_curr, cmd); % + add some noise here
    
end

plot(trajectory3(2,1:29),trajectory3(3,1:29),trajectory(2,1:10),trajectory(3,1:10),trajectory2(2,1:29),trajectory2(3,1:29));%,trajectory(1,1:2889),trajectory(3,1:2889));
%plot(trajectory2(2,1:29),trajectory2(3,1:29));
axis equal
function [updated_buffer, apply_cmd] = push(buffer,curr_cmd)
    updated_buffer = zeros(1,size(buffer,2));
    apply_cmd = buffer(1,size(buffer,2));
    for i = 2:size(buffer,2)
        updated_buffer(1,i) = buffer(1,i-1);
    end
    updated_buffer(1,1) = curr_cmd;
end