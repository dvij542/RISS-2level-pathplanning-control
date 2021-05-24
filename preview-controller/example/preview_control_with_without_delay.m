addpath('../src/')
addpath('../src/utils/')


%the usage is mostly same as tubeMPC
delay = 10;
Lr = 100;
buffer = zeros(1,delay);
%disp(buffer);
r = zeros(1,3250);
for i = 500:3000
    %time = i*deltaT;
    %r(1,i) = 1/sqrt(time-1)*sin(1.57*(time-2));
    r(1,i) = 1;
end
deltaT = 0.01;
A = [0 0 1; 0 0 -2; 0 2 -4];
B = [0; 2; 0];
D = [1; 0; 0];
Q = diag([1, 0,0]);
R = 0.15;
P = [0.725 0.1118 0.2155; 0.1118 0.0482 0.0464; 0.2155 0.0464 0.0910];
Am = diag([1,1,1]) + A*deltaT;
Bm = B*deltaT;
mysys = LinearSystem(Am, Bm, Q, R);
tao = delay*deltaT;
Km = inv(R)*transpose(B)*P*exp(A*tao);
Ke = Km(1);
Kx = Km(2:3);
Xc_vertex = [200, 200, 200; 200, 200, -200; 200, -200, -200; 200, -200, 200; -200, 200, 200; -200, 200, -200; -200, -200, -200; -200, -200, 200];
Uc_vertex = [100; -100];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);
e_sum = 0;
N_horizon = 20;
mpc = ModelPredictiveControl(mysys, Xc, Uc, N_horizon);

X = [0; 0; 0];
x_curr = [0; 0];
savedir_name = 'results';
Ac = A - B*inv(R)*transpose(B)*P;
cmd = 0;
trajectory = zeros(3,3000);
for i = 1:2990
    f1_ = 0;
    f2_ = 0;
    f3_ = 0;
    for j = i:(i+delay-1)
        f1_ = f1_ + exp(A*(i-j)*deltaT)*B*buffer(1,delay-j+i)*deltaT;
        f2_ = f2_ + exp(A*(i-j)*deltaT)*D*r(1,j)*deltaT;
    end
    f1 = -inv(R)*transpose(B)*P*exp(A*tao)*f1_;
    f2 = inv(R)*transpose(B)*P*exp(A*tao)*f2_;
    for j = (i+delay):(i+Lr-1)
        f3_ = f3_ + exp(A*(j-i-delay)*deltaT)*P*D*r(1,j)*deltaT;
    end
    f3 = inv(R)*transpose(B)*f3_;
    u_next = -Ke*e_sum -Kx*x_curr + f1 + f2 + f3;
    disp(u_next);
    [buffer, cmd] = push(buffer,u_next);
    %disp(cmd);
    X = mysys.propagate(X, cmd) - D*(r(1,i+1)-r(1,i)); % + add some noise here
    e_sum = e_sum + X(1)*deltaT;
    x_curr = x_curr + X(2:3)*deltaT;
    disp(x_curr(2));
    trajectory(1,i) = i;
    trajectory(2,i) = x_curr(2);
    %clf;
end

X = [0; 0; 0];
x_curr = [0; 0];
cmd = 0;
e_sum = 0;
buffer = zeros(1,delay);

for i = 1:2990
    f3_ = 0;
    for j = (i):(i+Lr-1)
        f3_ = f3_ + exp(A*(j-i)*deltaT)*P*D*r(1,j)*deltaT;
    end
    f3 = inv(R)*transpose(B)*f3_;
    u_next = -Ke*e_sum -Kx*x_curr + f3;
    [buffer, cmd] = push(buffer,u_next);
    disp(cmd);
    X = mysys.propagate(X, cmd) - D*(r(1,i+1)-r(1,i)); % + add some noise here
    e_sum = e_sum + X(1)*deltaT;
    x_curr = x_curr + X(2:3)*deltaT;
    disp(x_curr(2));
    trajectory(1,i) = i/100;
    trajectory(3,i) = x_curr(2);
end

plot(trajectory(1,1:2889),r(1,1:2889),trajectory(1,1:2889),trajectory(2,1:2889),trajectory(1,1:2889),trajectory(3,1:2889));

function [updated_buffer, apply_cmd] = push(buffer,curr_cmd)
    updated_buffer = zeros(1,size(buffer,2));
    apply_cmd = (buffer(1,size(buffer,2)-1) - buffer(1,size(buffer,2)))/0.01;
    for i = 2:size(buffer,2)
        updated_buffer(1,i) = buffer(1,i-1);
    end
    updated_buffer(1,1) = curr_cmd;
end