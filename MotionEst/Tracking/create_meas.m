function phi = create_meas()
%create some body fixed vectors
x_body(:,1) = [0;0];
x_body(:,2) = [20;20];
x_body(:,3) = [20;-20];

%create some origin motion
origin_motion = [linspace(100,800,floor(700/30));
                 100*ones(1,floor(700/30))];

%move the body points through the motion and assemble into a measurement
%matrix
meas = zeros(2*size(x_body,2),size(origin_motion,2));
for kk = 1:size(origin_motion,2)
    meas(:,kk) = reshape([eye(2),origin_motion(:,kk)]*[x_body;ones(1,size(x_body,2))],[],1);
end

%plot the measurements 
figure
hold on
plot(meas(1,:),meas(2,:),'b+',meas(3,:),meas(4,:),'b+',meas(5,:),meas(6,:),'b+')
%plot the connectivity
for kk = 1:size(meas,2)
    pts = [reshape(meas(:,kk),2,[]),meas(1:2,kk)];
    plot(pts(1,:)',pts(2,:)','-k')
end
axis equal
xlabel('X (pixels)')
ylabel('Y (pixels)')
title('Position of Image Measurements')

phi = meas;