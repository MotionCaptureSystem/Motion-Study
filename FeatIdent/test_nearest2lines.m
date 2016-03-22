%% Test nearest2lines

p1 = [1,1];
p2 = [.5,.5];
p3 = [.375,.75];

u3 = [1,1]/norm([1,1]);
u1 = [1,-1]/norm([1,-1]);
u2 = [1,0]/norm([1,0]);

lines = [p1,u1;p2,u2;p3,u3];

x = nearest2lines(lines);

figure
quiver(lines(:,1),lines(:,2),lines(:,3),lines(:,4))
hold on
plot(lines(:,1),lines(:,2),'*r',x(1),x(2),'+k')
axis equal



