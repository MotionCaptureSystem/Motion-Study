%% Test Script for Epipolar Line Gen

addpath '..\Common'
addpath '..\ImageProc'
fs_c = 119.88;

%% Generate a point
cams = [301,302];
%import camera parameters
load 'C:\SyntheticData\Synth1\CamStruct.mat'
H(:,:,1) = [eye(3),zeros(3,1);0,0,0,0];
cnt = 1;
for c = cams
    cnt = cnt+1;
    H(:,:,cnt) = Cam(c).H;
    Cam(c).sync_del = 0;
end
CFPlot(H,0.2)

%define a point start and stop position
X_start = [-1,0,-1.5]';
X_stop  = [-1,-1.5,1]';

plot3(X_start(1),X_start(2),X_start(3),'*r')
plot3(X_stop(1) ,X_stop(2) ,X_stop(3) ,'*b')
axis equal

%move linearly between start and stop
N = 30;
mX = (X_stop(1) - X_start(1))/N;
mY = (X_stop(2) - X_start(2))/N;
mZ = (X_stop(3) - X_start(3))/N;

start_frame = 10;
end_frame = 20;
t = start_frame:end_frame;

%% image the point in the camera
    figure
    hold on
for cc = cams
    %convert the points into camera basis
    Cam(cc).pts = [];
    pts_cam = [eye(3),zeros(3,1)]*invH(Cam(cc).H)*[mX*(t-Cam(cc).sync_del*119.88)+X_start(1);
                                                   mY*(t-Cam(cc).sync_del*119.88)+X_start(2);
                                                   mZ*(t-Cam(cc).sync_del*119.88)+X_start(3);
                                                   ones(1,size(t,2))];
    %Normalize by Z coord
    pts_ret(1,:) =   pts_cam(1,:)./pts_cam(3,:);
    pts_ret(2,:) =   pts_cam(2,:)./pts_cam(3,:);
    pts_ret(3,:) =   pts_cam(3,:)./pts_cam(3,:);
    %Distort Points
    pts_dist = ones(3,size(pts_ret,2));
    for pp = 1:size(pts_ret,2)
        pt_n = pts_ret(1:2,pp);
        r = norm(pt_n);
        kc = Cam(cc).kc;
        x_r = (1+kc(1)*r^2+kc(2)*r^4+kc(5)*r^6)*pt_n;
        dx  = [2*kc(3)*pt_n(1)*pt_n(2)+ kc(4)*(r^2+2*pt_n(1)^2);...
                2*kc(4)*pt_n(1)*pt_n(2)+kc(3)*(r^2+2*pt_n(2)^2)];
        pts_dist(1:2,pp) = x_r + dx;
    end
    %Add affine intrinisc params
    Cam(cc).pts = [eye(2),zeros(2,1)]*Cam(cc).K*pts_dist;
    plot(Cam(cc).pts(1,:)',Cam(cc).pts(2,:)')
    %remove points outside the camera FOV
    indx_x = Cam(cc).pts(1,:) < 0 | Cam(cc).pts(1,:)>1280;
    indx_y = Cam(cc).pts(2,:) < 0 | Cam(cc).pts(2,:)>720;
    cols = find(indx_x | indx_y);
    Cam(cc).pts(:,cols) = [];    
    
    %figure
    %plot(Cam(cc).pts(1,:)',Cam(cc).pts(2,:)','-r')
    Cam(cc).start_frame = start_frame;
    Cam(cc).end_frame = start_frame+size(Cam(cc).pts,2)-1;
    Cam(cc).frames = [];
end

%% 

timestep = 14;
cam1 = 301;
cam2 = 302;
pt = 1;
%determine the world time
t_world = timestep - floor(Cam(cam2).sync_del*fs_c);
dt_wc = 0;%1 - (Cam(cam2).sync_del*fs_c-floor(Cam(cam2).sync_del*fs_c));
%determine the time in the camera from which an epipolar line
%should be projected.
tt = t_world + floor(fs_c*Cam(cam1).sync_del)- Cam(cam1).start_frame + 1;

%if Cam(cc).sync_del
dt_wi = 0;%1 - (Cam(cam1).sync_del*fs_c-floor(Cam(cam1).sync_del*fs_c));
%else
%dt_wi = 0;
%end
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
p2_r = rm_distortion(pt2,Cam(cam2).K,Cam(cam2).fc,Cam(cam2).cc,Cam(cam2).alpha_c,Cam(cam2).kc);
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
phi2 = Cam(cam2).K\[Cam(cam2).pts(:,tt,pt);ones(1,size(p2_r,2))];
plot_ret_plane_inert(H2,phi2);

epi_line = epipolar_line(H1,H2,phi1);
x = linspace(-1,1,100);

figure 
hold on
for pp = 1:size(epi_line,1)
    line = epi_line(2)/epi_line(1)*(x-epi_line(3))+epi_line(4); 
    plot(phi2(1,pp)',phi2(2,pp),'+r',x,line,'-b')
end
