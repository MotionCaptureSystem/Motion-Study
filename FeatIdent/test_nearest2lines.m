%% Test nearest2lines
% All Lines Don't Intersect
p1 = [1,1];
p2 = [.5,.5];
p3 = [.375,.75];

u3 = -[1,1]/norm([1,1]);
u1 = -[1,-1]/norm([1,-1]);
u2 = -[1,0]/norm([1,0]);

lines = [p1,u1;p2,u2;p3,u3];

x = nearest2lines(lines);

figure
quiver(lines(:,1),lines(:,2),lines(:,3),lines(:,4))
hold on
plot(lines(:,1),lines(:,2),'*r',x(1),x(2),'+k')
axis equal


% All lines intersect at single point
p1 = [1,1];
p2 = [.7,.5];
p3 = [.375,.75];
pi = [.7,.7];

u1 = -(pi-p1)/norm(pi-p1);
u2 = -(pi-p2)/norm(pi-p2);
u3 = -(pi-p3)/norm(pi-p3);

lines = [p1,u1;p2,u2;p3,u3];

x = nearest2lines(lines);

figure
quiver(lines(:,1),lines(:,2),lines(:,3),lines(:,4))
hold on
plot(lines(:,1),lines(:,2),'*r',x(1),x(2),'+k')
axis equal
