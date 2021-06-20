function z = myStateFunction2(x,u,delta)
    tao = 1;
    z = 0;
    z(1) = tao*(u(1)-x(1));
    
end
