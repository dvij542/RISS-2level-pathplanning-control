plot(trajectory1(1,1:29),trajectory1(2,1:29),trajectory2(1,1:29),trajectory2(2,1:29),trajectory3(1,1:29),trajectory3(2,1:29));%,trajectory(2,1:10),trajectory(3,1:10),trajectory2(2,1:29),trajectory2(3,1:29));%,trajectory(1,1:2889),trajectory(3,1:2889));
title('Difference in path tracked for with delay consideration as constant vs variable with bounded uncertainity');
legend({'With variable time delay T=4 N=7 deltaT=0.3','With constant time delay T=6 N=7','Without time delay (Ideal MPC) T=0 N=7'},'Location','northeast')
saveas(gcf,'results/difference.png')
