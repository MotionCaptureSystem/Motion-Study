%% Test Script for Epipolar Line Gen
% load '..\CamStruct.mat'
% addpath '..\Common'

%% 
close all
t = 390;
cam1 = 318;
cam2 = 325;

p1 = Cam(cam1).pts_sync(:,t-Cam(cam1).start_frame+1:t-Cam(cam1).start_frame+20,1);
H1 = Cam(cam1).H;
phi1 = Cam(cam1).K\[p1;ones(1,size(p1,2))];
plot_ret_plane_inert(H1,phi1);

p2 = Cam(cam2).pts_sync(:,t-Cam(cam2).start_frame+1:t-Cam(cam2).start_frame+20,1);
H2 = Cam(cam2).H;
phi2 = Cam(cam2).K\[p2;ones(1,size(p2,2))];
plot_ret_plane_inert(H2,phi2);

epi_line = epipolar_line(H1,H2,phi1);
x = linspace(-1,1,100);

figure 
hold on
for pp = 1:size(epi_line,1)
    line = x*epi_line(pp,1)+epi_line(pp,2);
    plot(phi2(1,pp)',phi2(2,pp),'+r',x,line,'-b')
end
