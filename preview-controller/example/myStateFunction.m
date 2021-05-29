function z = myStateFunction(x,u,~)
    z = zeros(2,1);
    z(1) = 0.5*x(1) + 0.15*x(1)^2 + x(2) + 0.6*u(1);
    z(2) = x(1) - 0.2*x(2)^2 + 0.6*u(1);
end
