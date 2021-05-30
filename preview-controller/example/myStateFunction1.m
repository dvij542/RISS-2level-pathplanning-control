function z = myStateFunction1(x,u,delta)
    tao = 1;
    A = [-6.68 -0.81956 -1.98; 1.663 -4.07147 -6.69733; 0 0 -tao];
    B = [0; 0; tao];

    z = A*x + B*u;
    
end
