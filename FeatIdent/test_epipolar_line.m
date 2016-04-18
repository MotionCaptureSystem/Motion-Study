%% Test Script for Epipolar Line Gen
load 'C:\SyntheticData\Synth1\CamStruct.mat'
addpath '..\Common'
addpath '..\ImageProc'
fs_c = 119.88;
%% 
close all
timestep = 15;   %timestep in camera which the epipolar line will be plotted
cam1 = 309;      %camera to project from 
cam2 = 318;      %camera to project to
pt = 1;          %point number

%determine the world time
t_world = timestep - floor(Cam(cam2).sync_del*fs_c);
if Cam(cam2).sync_del
    dt_wc = 1 - (Cam(cam2).sync_del*fs_c-floor(Cam(cam2).sync_del*fs_c));
else
    dt_wc = 0;
end
%determine the time in the camera from which an epipolar line
%should be projected.
tt = t_world + floor(fs_c*Cam(cam1).sync_del)- Cam(cam1).start_frame + 1;

if Cam(cam1).sync_del
    dt_wi = 1 - (Cam(cam1).sync_del*fs_c-floor(Cam(cam1).sync_del*fs_c));
else
    dt_wi = 0;
end
dt_ci = dt_wc - dt_wi;
d = sign(dt_ci);

%if the points are availabe in this camera
if dt_ci < 0 
    pt1 = Cam(cam1).pts(:,tt+d,pt);
    pt2 = Cam(cam1).pts(:,tt,pt);
else
    pt1 = Cam(cam1).pts(:,tt,pt);
    pt2 = Cam(cam1).pts(:,tt+d,pt);
end

%remove distortion from the points 
p1_r = rm_distortion(pt1,Cam(cam1).K,Cam(cam1).fc,Cam(cam1).cc,Cam(cam1).alpha_c,Cam(cam1).kc);%rm_distortion(x_p, K, fc, prin_p, skew, dist_c)
p2_r = rm_distortion(pt2,Cam(cam1).K,Cam(cam1).fc,Cam(cam1).cc,Cam(cam1).alpha_c,Cam(cam1).kc);
%interpolate the point in undist pix coords
if dt_ci < 0 
    p_r = p1_r + (p2_r - p1_r)*(1+dt_ci);
else
    p_r = p1_r + (p2_r - p1_r)*dt_ci;
end

H1 = Cam(cam1).H;
phi1 = Cam(cam1).K\[p_r;ones(1,size(p_r,2))];
plot_ret_plane_inert(H1,phi1);

H2 = Cam(cam2).H;
phi2 = Cam(cam2).pts(:,timestep-Cam(cc).start_frame+1,pt);
phi2 = Cam(cam2).K\[rm_distortion(phi2,Cam(cam2).K,Cam(cam2).fc,Cam(cam2).cc,Cam(cam2).alpha_c,Cam(cam2).kc);1];
plot_ret_plane_inert(H2,phi2);

epi_line = epipolar_line(H1,H2,phi1);
x = linspace(-1,1,100);

figure 
hold on
for pp = 1:size(epi_line,1)
    line = epi_line(2)/epi_line(1)*(x-epi_line(3))+epi_line(4); 
    plot(phi2(1,pp)',phi2(2,pp),'+r',x,line,'-b')
end
axis equal