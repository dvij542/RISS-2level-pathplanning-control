function z = myStateFunction2(x,u,delta)
    tao = 0;
    A = [-6.68 -0.81956 0; 1.663 -4.07147 0; 0 0 -tao];
    B = [-1.98; -6.69733; tao];
    
    z = A*x + B*u;
    
end
