nx = 3;
ny = 1;
nu = 1;
nlobj = nlmpc(nx,ny,nu);
Ts = 0.1;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 20;
nlobj.ControlHorizon = 20;
nlobj.Model.StateFcn = "myStateFunction1";
nlobj.Model.IsContinuousTime = true;
nlobj.Model.OutputFcn = @(x,u,params) [x(2)];
nlobj.Weights.OutputVariables = [0.8];
nlobj.Weights.ManipulatedVariables = [0.2];
nlobj.Optimization.ReplaceStandardCost = false;
A = [-6.68 -0.81956 ; 1.663 -4.07147];
B = [-1.98;-6.69733];
for ct = 1:nu
    nlobj.MV(ct).Min = -2;
    nlobj.MV(ct).Max = 2;
end

ref = zeros(325,1);
for i = 50:300
    %time = i*deltaT;
    %r(1,i) = 0.3;
%     r(1,i) = 1/sqrt(time-1)*sin(1.57*(time-2));
    if i<100
        ref(i,1) = 0.3;
    end
    if i>=100 & i<150
        ref(i,1) = -0.3;
    end
    if i>=150 & i<200
        ref(i,1) = 0.3;
    end
end

%nlobj.Model.NumberOfParameters = 1;
%nlobj.Optimization.CustomCostFcn = "myCostFunction2";
tao = 1;
x_curr = [0; 0; 0];
cmd = 0;
u_next = 0;
trajectory = zeros(2,300);
for i = 1:279
    trajectory(1,i) = i;
    trajectory(2,i) = x_curr(2);
    x_new = x_curr;
    u_next = nlmpcmove(nlobj,x_new,u_next,ref(i:(i+20),:));
    %disp(u_next);
    cmd = cmd + tao*(u_next-cmd)*Ts;
    disp(cmd);
    x_curr(1) = x_curr(1) + Ts*(A(1,1)*x_curr(1)+A(1,2)*x_curr(2)+B(1)*x_curr(3));
    x_curr(2) = x_curr(2) + Ts*(A(2,1)*x_curr(1)+A(2,2)*x_curr(2)+B(2)*x_curr(3));
    x_curr(3) = cmd;
    
end

tao = 1;
x_curr = [0; 0; 0];
cmd = 0;
u_next = 0;
trajectory2 = zeros(2,300);
nlobj.Model.StateFcn = "myStateFunction2";
for i = 1:279
    trajectory2(1,i) = i;
    trajectory2(2,i) = x_curr(2);
    x_new = x_curr;
    u_next = nlmpcmove(nlobj,x_new,u_next,ref(i:(i+20),:));
    %disp(u_next);
    cmd = cmd + tao*(u_next-cmd)*Ts;
    disp(cmd);
    x_curr(1) = x_curr(1) + Ts*(A(1,1)*x_curr(1)+A(1,2)*x_curr(2)+B(1)*x_curr(3));
    x_curr(2) = x_curr(2) + Ts*(A(2,1)*x_curr(1)+A(2,2)*x_curr(2)+B(2)*x_curr(3));
    x_curr(3) = cmd;
    
end

nx = 1;
ny = 1;
nu = 1;
nlobj2 = nlmpc(nx,ny,nu);
Ts = 0.1;
nlobj2.Ts = Ts;
nlobj2.PredictionHorizon = 20;
nlobj2.ControlHorizon = 20;
nlobj2.Model.StateFcn = "myStateFunction_low";
nlobj2.Model.IsContinuousTime = true;
nlobj2.Weights.OutputVariables = [1.0];
nlobj2.Weights.ManipulatedVariables = [0.15];
nlobj2.Optimization.ReplaceStandardCost = false;
for ct = 1:nu
    nlobj.MV(ct).Min = -2;
    nlobj.MV(ct).Max = 2;
end

tao = 1;
x_curr = [0; 0; 0];
cmd = 0;
u_next = 0;
trajectory3 = zeros(2,300);

for i = 1:279
    trajectory3(1,i) = i;
    trajectory3(2,i) = x_curr(2);
    x_new = x_curr;
    [~,~,info] = nlmpcmove(nlobj,x_new,u_next,ref(i:(i+20),:));
    %disp(u_next);
    u_next = nlmpcmove(nlobj2,cmd,u_next,info.MVopt);
    cmd = cmd + tao*(u_next-cmd)*Ts;
    disp(cmd);
    x_curr(1) = x_curr(1) + Ts*(A(1,1)*x_curr(1)+A(1,2)*x_curr(2)+B(1)*x_curr(3));
    x_curr(2) = x_curr(2) + Ts*(A(2,1)*x_curr(1)+A(2,2)*x_curr(2)+B(2)*x_curr(3));
    x_curr(3) = cmd;
    
end


plot(trajectory(1,1:279),ref(1:279,1),trajectory(1,1:279),trajectory(2,1:279),trajectory2(1,1:279),trajectory2(2,1:279),trajectory3(1,1:279),trajectory3(2,1:279));

function [updated_buffer, apply_cmd] = push(buffer,curr_cmd)
    updated_buffer = zeros(1,size(buffer,2));
    apply_cmd = buffer(1,size(buffer,2));
    for i = 2:size(buffer,2)
        updated_buffer(1,i) = buffer(1,i-1);
    end
    updated_buffer(1,1) = curr_cmd;
end



