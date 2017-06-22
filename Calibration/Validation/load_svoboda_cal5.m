function camstruct = load_svoboda_cal4(options)

%cams   -A row vector of the cameras to import 

%distortion_coeff           = kc
%distortion_coeff_error     = kc_error
%focal_length               = fc
%focal_length_error         = fc_error
%principal_point            = cc
%principal_point_error      = cc_error
%skew                       = alpha_c
%skew_error                 = alpha_c_error
cams = options.cams;
ncam = length(cams);
pts = options.pts;
npts = length(pts);
tstart = options.tstart;
tstop = options.tstop;
timesteps = linspace(tstart,tstop,(tstop-tstart)+1);

default_dir = pwd;
plot_flag = options.plotflag;

%Determine the files in this directory
list = dir(options.path);
%initialize index vars
mm = 1;

%if the global frame is 1 then the 
load([options.path,filesep,'CalVal10_06102016.mat'],'linear','C','R','cam','inliers')

Ws = linear.Ws;
load([options.path,filesep,'Ce.dat']);
load([options.path,filesep,'Re.dat']);
load([options.path,filesep,'points.dat']);
load([options.path,filesep,'Pmatrices.dat']);
for cc = 1:ncam
    index = find([cam.camId] == cams(cc));
    cam_num = cams(cc);
    %Load parameters to remove distortions       
    [K,kc] = readradfile([options.path,filesep,'Cam',num2str(cam_num),'.rad']);
    camstruct(cam_num).K_dist = K;
    camstruct(cam_num).kc = kc;
    %Convert Pmat to KRt
    [K, ~, ~] = Pmat2KRt(Pmatrices(3*index-2:3*index,:));
    Rot = Re(3*index-2:3*index,:);
    t = Ce(:,index);
    camstruct(cam_num).K = K;
    camstruct(cam_num).H = [Rot,-t;0,0,0,1];
    %Load the original distorted points
    camstruct(cam_num).pts_dist  = points(3*index-2:3*index-1,:);

    fclose('all');
end
