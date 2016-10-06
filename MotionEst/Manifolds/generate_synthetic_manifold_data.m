%GENERATE_SYNTHETIC_MANIFOLD_DATA      -This funciton creates synthetic
%manifold data for use with the MotionStudy estimation code.  This script
%requires [x,y,z] cooridnates of N points stored in a [3N x K] matrix
%called POINTS.  K is the number of timesteps and N is the number of
%points.  Specified within this file are the camera parameters and numbers
%of cameras to be used for measurements.  Noise may also be added within
%this script.  The output is a CamStruct.mat file which can be imported
%into the MotionStudy Code.
clear all 
close all

addpath D:\Users\Matt\Documents\GitHub\MotionStudy\Common
addpath D:\Users\Matt\Documents\GitHub\MotionStudy\MotionEst\Models
%% Load the Data
load('ManifoldPts.mat')
[npts,nsteps] = size(points);
npts = npts/3;
colors = hsv(npts);
figure
hold on
for pp = 1:npts 
    plot3(points(3*(pp-1)+1,:)',points(3*(pp-1)+2,:)',points(3*pp,:)','color',colors(pp,:))
end
xlabel('x')
ylabel('y')
zlabel('z')

%% Define Camera Parameters
%Detefine Extrinsic Parameters
ncam  = 2;
nring = 1;
[S,~,~] = svd(points);
H(:,:,1) = [S,S*[-1;0;-3];
            0,0,0,1];
H(:,:,2) = [S,S*[1;0;-3];
            0,0,0,1];
%Create the camera structure from previous GoPro Intrinsics        
for c = 1:ncam
    load(['C:\ShandongData2016\BatFlight_20160717\Calibration_run\Intrinsic\CalTech',filesep,'Cam',num2str(c+300),filesep,'int_cam',num2str(c+300),'.mat'],'KK','kc','cc','fc','alpha_c')
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
    Cam(c).H = H(:,:,c);
end

%plot the camera positions on the points plot
CFPlot(H,0.2);

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

%% Measure 3D feature points using a camera model.
%create measurements using pinhole camera model
cam_points = zeros(npts*ncam*2,nsteps);
for kk = 1:nsteps
    [cam_points(:,kk)] = CamNet(points(:,kk),Cam);
end

for cc = 1:ncam
    for pp = 1:npts
        Cam(cc).pts(:,:,pp) = cam_points((cc-1)*2*npts+2*(pp-1)+1:(cc-1)*2*npts+2*(pp-1)+2,:);
    end
    Cam(cc).sync_del = 0;
end

%% Plot The Camera Measurements
ii = 0;
figure
for cc = 1:ncam
    ii = ii+1;
    for pp = 1:npts
        subplot(2,1,ii);
        hold on 
        plot(Cam(cc).pts(1,:,pp)',Cam(cc).pts(2,:,pp)','color',colors(pp,:))
        %text(Cam(cc).pts(1,1,pp)'-1.2*pp-2,Cam(cc).pts(2,1,pp)',num2str(pp))
    end
    clear title xlabel ylabel
    title(sprintf('Cam %d',cc))
    xlabel = ('X (pixels)');
    ylabel = ('Y (pixels)');
end

save('CamStruct.mat','Cam')
ukf.Features = points;
ukf.X = points;
save('EstStructSynth.mat','ukf')
