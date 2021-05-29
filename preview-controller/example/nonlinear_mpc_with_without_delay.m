nx = 2;
ny = 2;
nu = 1;
nlobj = nlmpc(nx,ny,nu);
Ts = 0.075;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 100;
nlobj.ControlHorizon = 100;
nlobj.Model.StateFcn = "myStateFunction";
nlobj.Model.IsContinuousTime = true;
nlobj.Optimization.ReplaceStandardCost = true;
for ct = 1:nu
    nlobj.MV(ct).Min = -2;
    nlobj.MV(ct).Max = 2;
end
%nlobj.Model.NumberOfParameters = 1;
nlobj.Optimization.CustomCostFcn = "myCostFunction";
delay = 5;
x_curr = [1; -1];
buffer = zeros(1,delay);
cmd = 0;
trajectory = zeros(3,30);
for i = 1:79
    trajectory(1,i) = i;
    trajectory(2,i) = x_curr(1);
    trajectory(3,i) = x_curr(2);
    x_new = x_curr;
    for j = 1:delay
        u_curr = buffer(delay+1-j);
        x_new(1) = x_new(1) + Ts*(0.5*x_new(1) + 0.15*x_new(1)^2 + x_new(2) + 0.6*u_curr);
        x_new(2) = x_new(2) + Ts*(x_new(1)-0.2*x_new(2)^2+0.6*u_curr);
    end
    u_next = nlmpcmove(nlobj,x_new,u_next);
    disp(u_next);
    [buffer, cmd] = push(buffer,u_next);
    x_curr(1) = x_curr(1) + Ts*(0.5*x_curr(1) + 0.15*x_curr(1)^2 + x_curr(2) + 0.6*cmd);
    x_curr(2) = x_curr(2) + Ts*(x_curr(1)-0.2*x_curr(2)^2+0.6*cmd);
end

x_curr = [1; -1];
buffer = zeros(1,delay);
cmd = 0;
trajectory1 = zeros(3,30);
for i = 1:79
    trajectory1(1,i) = i;
    trajectory1(2,i) = x_curr(1);
    trajectory1(3,i) = x_curr(2);
    x_new = x_curr;
    u_next = nlmpcmove(nlobj,x_new,u_next);
    disp(u_next);
    cmd = u_next;
    [buffer, cmd] = push(buffer,u_next);
    x_curr(1) = x_curr(1) + Ts*(0.5*x_curr(1) + 0.15*x_curr(1)^2 + x_curr(2) + 0.6*cmd);
    x_curr(2) = x_curr(2) + Ts*(x_curr(1)-0.2*x_curr(2)^2+0.6*cmd);
end

x_curr = [1; -1];
buffer = zeros(1,delay);
cmd = 0;
trajectory2 = zeros(3,30);
for i = 1:79
    trajectory2(1,i) = i;
    trajectory2(2,i) = x_curr(1);
    trajectory2(3,i) = x_curr(2);
    x_new = x_curr;
    u_next = nlmpcmove(nlobj,x_new,u_next);
    disp(u_next);
    cmd = u_next;
    x_curr(1) = x_curr(1) + Ts*(0.5*x_curr(1) + 0.15*x_curr(1)^2 + x_curr(2) + 0.6*cmd);
    x_curr(2) = x_curr(2) + Ts*(x_curr(1)-0.2*x_curr(2)^2+0.6*cmd);
end

plot(trajectory(2,1:79),trajectory(3,1:79),trajectory1(2,1:79),trajectory1(3,1:79),trajectory2(2,1:79),trajectory2(3,1:79));
axis equal
function [updated_buffer, apply_cmd] = push(buffer,curr_cmd)
    updated_buffer = zeros(1,size(buffer,2));
    apply_cmd = buffer(1,size(buffer,2));
    for i = 2:size(buffer,2)
        updated_buffer(1,i) = buffer(1,i-1);
    end
    updated_buffer(1,1) = curr_cmd;
end



