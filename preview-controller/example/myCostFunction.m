function J = myCostFunction(x,u,~,~,~)
    J = 0;
    for i = 1:100
        J = J + 0.5*x(i,1)^2 + 0.5*x(i,2)^2 + 1*u(i,1)^2;
    end 
end
