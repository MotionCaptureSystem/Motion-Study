function camstruct = load_svoboda_cal4(options)

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
%load([options.path,filesep,'CalVal10_06102016.mat'],'cam')

load([options.path,filesep,'Ce.dat']);
load([options.path,filesep,'Re.dat']);
load([options.path,filesep,'points.dat']);
for cc = 1:ncam
    index_im2pts = find(options.cams_im2pts == cams(cc));
    index_cal  = find(options.cams_cal == cams(cc));
    cam_num = cams(cc);
    %load and store extrinsics
    t = -Ce(:,index_cal);
    Rot = Re(3*index_cal-2:3*index_cal,:)';
    camstruct(cam_num).H = [Rot,t;0,0,0,1];
    %Load and store distortion parameters
    [K,kc] = readradfile_mb(['C:\ShandongData2016\CalVal11_06142016\Calibration_run\Intrinsic\CalTech',filesep,'Cam',num2str(cam_num),filesep,'int_cam',num2str(cam_num),'.mat']);
    camstruct(cam_num).K_dist = K; %note that this is a different K than the one used for triangulation
    camstruct(cam_num).kc = kc;
    
    %Open the .cal file
    fid  = fopen([options.path,filesep,'camera',num2str(cam_num),'.cal']);
    data = textscan(fid,'%s%s%f');      %Read the data
    %A hard lesson learned: K matrix from cal file must be negated
    camstruct(cam_num).K = -[data{3}(13:15)';data{3}(16:18)';data{3}(19:21)'];
    
    %camstruct(cam_num).idin = cam(index).idin;
    %npts = size(points,2);
    %all_pt_nums = 1:npts;
    %outliers = setdiff(all_pt_nums,cam(index).idin);
    camstruct(cam_num).pts_dist  = points(3*index_im2pts-2:3*index_im2pts-1,:);
    %camstruct(cam_num).pts_dist(:,outliers)  = NaN;

    fclose('all');
end
