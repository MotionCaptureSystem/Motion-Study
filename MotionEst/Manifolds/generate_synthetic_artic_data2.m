%function generate_synthetic_artic_data()
%GENERATE_SYNTHETIC_ARTIC_DATA      -This funciton creates synthetic
%articulated skeleton data for developing recursive Bayesian estimation
%techniques for studying bat flight.  Tunnel dimensions are set within this
%script, 3d points are generated, and camera measurements are taken.

fprintf('.........................Frame Dimensions.....................\n')
l = 5000;          %length of tunnel
w = 2500;        %width of tunnel
h = 2500;          %height of tunnel
fprintf('Height = %d, Width = %d, Length = %d.\n',h,w,l);

dead_z = 600;     %dead zone on each end of tunnel (no cams or points here)
cam_off = 60; %cams can't be mounted directly on frame so offset by this
point_off = 1000;%points shouldn't go behind cams so offset by this much.

fprintf('Dead Zone at Each end of Tunnel Is %d [mm] along length.\n',dead_z)

%% Define the Bat Skeleton and path
SkeletonDefn

nsteps = 400;
t = linspace(0,.32,nsteps);
% %Body Translation
% q(2,:)  = linspace(1500,3500,nsteps);
% q(1,:)  = 1250*ones(1,nsteps);
% q(3,:)  = 1000*ones(1,nsteps);
% %Body Rotation
% q(4,:)  = zeros(1,nsteps);
% q(5,:)  = pi/4*sin(linspace(0,4*pi,nsteps));
% q(6,:)  = pi/4*ones(1,nsteps);
% %Humerus Rotation
% q(7,:)  = -pi/2*ones(1,nsteps);%pi/4*sin(linspace(0,4*pi,nsteps))-pi/2;
% q(8,:)  = -pi/2*ones(1,nsteps);%pi/4*sin(linspace(0,4*pi,nsteps))-pi/2;
% q(9,:)  = pi/4*sin(linspace(0,4*pi,nsteps))+pi/2;
% %Radius Rotation
% q(10,:)  = pi/4*sin(linspace(0,4*pi,nsteps));
% %Metacarpal 1
% q(11,:)  = 0*ones(1,nsteps);%-pi/8*sin(linspace(0,4*pi,nsteps))+pi/8;
% q(12,:)  = pi/8*sin(linspace(0,4*pi,nsteps))+pi/8;
% %Metacarpal 2
% q(13,:)  = pi/8*ones(1,nsteps);%pi/8*sin(linspace(0,4*pi,nsteps))+pi/8;
% q(14,:)  = pi/8*sin(linspace(0,4*pi,nsteps))+pi/8;
% %Metacarpal 3
% q(15,:)  = pi/4*ones(1,nsteps);%-pi/8*sin(linspace(0,4*pi,nsteps))+pi/8;
% q(16,:)  = pi/8*sin(linspace(0,4*pi,nsteps))+pi/8;
%Body Translation
q(2,:)  = linspace(1500,3500,nsteps);
q(1,:)  = 1250*ones(1,nsteps);
q(3,:)  = 1000*ones(1,nsteps);
%Body Rotation
q(4,:)  = zeros(1,length(t));
q(5,:)  = pi/8*sin(linspace(0,4*pi,nsteps));
q(6,:)  = pi/2*ones(1,nsteps);
%Humerus Rotation
q(7,:)  = -20/180*pi*cos(2*t/t(end)*2*pi)+130/180*pi;%pi/4*sin(linspace(0,4*pi,nsteps))-pi/2;
q(8,:)  = -30/180*pi*cos(2*t/t(end)*2*pi)-90/180*pi;%pi/4*sin(linspace(0,4*pi,nsteps))-pi/2;
q(9,:)  = 0*(-40/180*pi*cos(2*t/t(end)*2*pi))-90*pi/180;
%Radius Rotation
q(10,:)  = -pi/4*sin(linspace(0,4*pi,nsteps))+0;
%Metacarpal 1
q(11,:)  = 30/180*pi*ones(1,nsteps);%-pi/8*sin(linspace(0,4*pi,nsteps))+pi/8;
q(12,:)  = -pi/8*sin(linspace(0,4*pi,nsteps))+-pi/8;
%Metacarpal 2
q(13,:)  = 70/180*pi*ones(1,nsteps);%pi/8*sin(linspace(0,4*pi,nsteps))+pi/8;
q(14,:)  = -pi/8*sin(linspace(0,4*pi,nsteps))+-pi/8;
%Metacarpal 3
q(15,:)  = 120*pi/180*ones(1,nsteps);%-pi/8*sin(linspace(0,4*pi,nsteps))+pi/8;
q(16,:)  = -pi/8*sin(linspace(0,4*pi,nsteps))+-pi/8;
%create the body basis transforms and the coordinate transforms for each
%time step2
dof = 0;
for ll = 1:length(synthConfig.link)
    nDof = synthConfig.link(ll).nDof;
    dof = dof+synthConfig.link(ll).nDof;
    for kk = 1:nsteps
        q_lk = q(dof-nDof+1:dof,kk);
        if strcmp(synthConfig.link(ll).IDkern,'DH')
            kinc(kk).link(ll).H=DHTransforms(q_lk+synthConfig.link(ll).thetas,...
                        synthConfig.link(ll).alphas,...
                        synthConfig.link(ll).disps,...
                        synthConfig.link(ll).offsets);
        elseif  strcmp(synthConfig.link(ll).IDkern,'YPR')
            theta = q_lk(4:6);
            d = q_lk(1:3);
            kinc(kk).link(ll).H = YPRTransform(theta, d);
        end
    end
end

npts = size([synthConfig.link.BFvecs],2);
features = zeros(3*npts,nsteps);
n_pts_tot = 0;
for ll = 1:length(synthConfig.link)
    n_bf_pts = size(synthConfig.link(ll).BFvecs,2);
    n_pts_tot = n_pts_tot+n_bf_pts;
    for kk = 1:nsteps
    X = [eye(3,3),zeros(3,1)]*hnode2node(kinc(kk),synthConfig,1,ll)*[synthConfig.link(ll).BFvecs;ones(1,n_bf_pts)];
    features(3*(n_pts_tot-n_bf_pts)+1:3*n_pts_tot,kk) = X(:);
    end
    %assoc{ll}(:,(n_pts_tot-n_bf_pts)+1:n_pts_tot) = [ll*ones(1,n_bf_pts);1:n_bf_pts];
end

%% Define Camera Parameters
%Detefine Extrinsic Parameters
ncam = 40;
nring = 5;
%evenly space rings in y 
y_cam = linspace(dead_z, l-dead_z,nring);
%evenly space cams around each ring
x_cam = linspace(w-cam_off, cam_off, 3);
z_cam = linspace(cam_off, h-cam_off, 3);
%collect camera coords
c = 0;
cam_pos = zeros(3,ncam);
for cc = 1:ncam/nring
    if any(find([1,4,6] == cc))
        cam_pos(1,cc:ncam/nring:ncam) = x_cam(1);
    elseif any(find([2,7] == cc))
        cam_pos(1,cc:ncam/nring:ncam) = x_cam(2);
    else
        cam_pos(1,cc:ncam/nring:ncam) = x_cam(3);
    end
    
    if any(find([1,2,3] == cc))
        cam_pos(3,cc:ncam/nring:ncam) = z_cam(3);
    elseif any(find([4,5] == cc))
        cam_pos(3,cc:ncam/nring:ncam) = z_cam(2);
    else
        cam_pos(3,cc:ncam/nring:ncam) = z_cam(1);
    end
end

for rr = 1:nring
    cam_pos(2,(rr-1)*8+1:rr*8) = y_cam(rr);
end

%specify camera orientations
%epsilon = elevation angle (tip)
%alpha  = azimuth angle (rotation)

%assume the fifth camera in each ring is oriented wrt zero frame by 
R = zeros(3,3,ncam);
for cc = 5:8:40
    R(:,:,cc) = [0,0,1; 1,0,0; 0,1,0];
end
%apply elevation rotation about fifth camera x-axis
ep = linspace(pi/4,7*pi/4,ncam/nring-1)+pi/40*rand(1,ncam/nring-1);
cam_id = [3,2,1,4,6,7,8];
for cc = 1:length(ep)
    for zz = cam_id(cc):8:40
        R(:,:,zz) = R(:,:,5)*[1,0,0;0,cos(ep(cc)+pi/40),-sin(ep(cc)+pi/40);0,sin(ep(cc)+pi/40),cos(ep(cc)+pi/40)];
    end
end

%apply azimuth rotations about each cameras y axis
a = 20/180*pi;
alpha = linspace(a,-a,nring);
H = zeros(4,4,ncam);
for rr = 1:nring
    Ry = [cos(alpha(rr)+pi/40),0,sin(alpha(rr)+pi/40);0,1,0;-sin(alpha(rr)+pi/40),0,cos(alpha(rr)+pi/40)];
    for cc = (rr-1)*ncam/nring+1:rr*ncam/nring
        R(:,:,cc) = R(:,:,cc)*Ry;
        H(:,:,cc) = [R(:,:,cc),cam_pos(:,cc);0,0,0,1];
        Cam(cc).H = H(:,:,cc);
    end
end

figure
%CFPlot(H,500);

hold on
%plot3(cam_pos(1,:)',cam_pos(2,:)',cam_pos(3,:)', 'ob')
linespec = {'-r','-b','-g', '-m','-k','-c','--b','-r','-b','-g', '-m','-k','-c','--r','+--b','+--g'};
colors = hsv(15);
for pp = 1:15
    plot3(features(3*(pp-1)+1,:)', features(3*(pp-1)+2,:)', features(3*(pp-1)+3,:)', 'color', colors(pp,:), 'LineWidth', 2)
    %text(features(3*(pp-1)+1,1)'+5 + 1.5*pp, features(3*(pp-1)+2,1)', features(3*(pp-1)+3,1)', num2str(pp))
end

%CFPlot(H,500);
plot_kin_chain(kinc,synthConfig,[1:5:size(q,2)])
axis([800,1600,1500,3500,700,1300]) 
h = gca;
options.plot.fig_txt_props  = {'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'Bold'};
set(h,options.plot.fig_txt_props{:})
textobj = findobj(h, 'type', 'text');
lineobj = findobj(h, 'type', 'line');
set(lineobj, 'LineWidth', 2);
set(textobj,  options.plot.fig_txt_props{:});
clear title xlabel ylabel
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]'); 
axis equal

%Define Intrinsic Parameters
%load int_cam301.mat
for c = 1:ncam
    load(['.',filesep,'Calibration_run',filesep,'Intrinsic',filesep,'CalTech',filesep,'Cam',num2str(c+300),filesep,'int_cam',num2str(c+300),'.mat'],'KK','kc','cc','fc','alpha_c')
    Cam(c).K = KK;
    Cam(c).K_dist = KK;
    Cam(c).kc_dist = zeros(1,5);
    Cam(c).cc = cc;
    Cam(c).cc_dist = cc;
    Cam(c).fc = fc;
    Cam(c).fc_dist = fc;
    Cam(c).alpha_c = alpha_c;
    Cam(c).alpha_c_dist = alpha_c;
    Cam(c).Res = [720,1280];
    Cam(c).start_frame = 1;
    Cam(c).end_frame = nsteps;
end


%% Measure 3D feature points using a camera model.
%create measurements using pinhole camera model
points = zeros(npts*ncam*2,nsteps);
for kk = 1:nsteps
    [points(:,kk)] = CamNet(features(:,kk),Cam)+2*randn(npts*ncam*2,1);
end

for cc = 1:ncam
    for pp = 1:npts
        Cam(cc).pts(:,:,pp) = points((cc-1)*2*npts+2*(pp-1)+1:(cc-1)*2*npts+2*(pp-1)+2,:);
    end
    Cam(cc).pt_assoc = {[1,2,3,4,5],[6,3],[8,6],[10,8],[12,8],[14,8]};
    Cam(cc).sync_del = 0;
end

ii = 0;
figure
for cc = 1:5:ncam
    ii = ii+1;
    for pp = 1:npts
        subplot(2,4,ii);
        hold on 
        plot(Cam(cc).pts(1,:,pp)',Cam(cc).pts(2,:,pp)',linespec{pp})
        text(Cam(cc).pts(1,1,pp)'-1.2*pp-2,Cam(cc).pts(2,1,pp)',num2str(pp))
    end
    clear title xlabel ylabel
    title(sprintf('Cam %d',cc))
    xlabel = ('X (pixels)');
    ylabel = ('Y (pixels)');
end

save('CamStruct.mat','Cam')
ukf.X = q;
ukf.Features = features;
save('EstStructSynth.mat','ukf','kinc')
