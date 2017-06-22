%% Debug Script for EPIPOLAR_LINE
clear all
close all
addpath ..\Common
%Define Camera Extrinsics
H1 = YPRTransform([-pi/8,0,0],[0,0,0]);
H2 = YPRTransform([pi/8,0,0],[0,5,0]);
H3 = YPRTransform([0,-pi/8,0],[2.5,2.5,0]);
H(:,:,1) = H1;
H(:,:,2) = H2;
H(:,:,3) = H3;
%CFPlot(H,0.5);
%axis equal

%define point in inertial basis 
X = [2,2.5,5]';
%plot3(X(1),X(2),X(3),'*r')

%determine X in camera bases
X_1 = [eye(3),zeros(3,1)]*invH(H1)*[X;1];
X_2 = [eye(3),zeros(3,1)]*invH(H2)*[X;1];
X_3 = [eye(3),zeros(3,1)]*invH(H3)*[X;1];
%Determine Retinal Coords of X in each camera
phi1 = X_1/X_1(3);
phi2 = X_2/X_2(3);
phi3 = X_3/X_3(3);

%plot the complete configuration
plot_ret_plane_inert(H1,phi1)
plot_ret_plane_inert(H2,phi2)
plot_ret_plane_inert(H3,phi3)
plot3(X(1),X(2),X(3),'*r')
pts = [X';H1(1:3,4)';H2(1:3,4)';X'];
plot3(pts(:,1),pts(:,2),pts(:,3),'-b');

%compute the epipolar lines in Cam 1
epi = epipolar_line(H2,H1,phi2);
epi2 = epipolar_line(H3,H1,phi3);
x = linspace(-1,1,10);
y1 = epi(1)*x+epi(2);
y2 = epi2(1)*x+epi2(2);
figure
plot(x,y1,x,y2,phi1(1),phi1(2),'+r'); title('Cam 1 Ret Plane'); 
axis([-1,1,-1,1])
legend('Cam 2 Epi', 'Cam 3 Epi', 'phi', 'Location','Best');

%compute the epipolar lines in Cam 2
epi = epipolar_line(H1,H2,phi1);
epi2 = epipolar_line(H3,H2,phi3);
x = linspace(-1,1,10);
y1 = epi(1)*x+epi(2);
y2 = epi2(1)*x+epi2(2);
figure
plot(x,y1,x,y2,phi2(1),phi2(2),'+r'); title('Cam 2 Ret Plane'); 
axis([-1,1,-1,1])
legend('Cam 1 Epi', 'Cam 3 Epi', 'phi', 'Location','Best');

%compute the epipolar lines in Cam 3
epi = epipolar_line(H1,H3,phi1);
epi2 = epipolar_line(H2,H3,phi2);
x = linspace(-1,1,10);
y1 = epi(1)*x+epi(2);
y2 = epi2(1)*x+epi2(2);
figure
plot(x,y1,x,y2,phi3(1),phi3(2),'+r'); title('Cam 3 Ret Plane'); 
axis([-1,1,-1,1])
legend('Cam 1 Epi', 'Cam 2 Epi', 'phi', 'Location','Best');



rmpath ..\Common