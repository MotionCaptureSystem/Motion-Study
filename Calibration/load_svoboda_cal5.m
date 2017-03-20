function [options, camstruct] = load_svoboda_cal5(camstruct,options)

cams = options.cams;
ncam = length(cams);

if ~isfield(options,'cams_im2pts')
    options.cams_im2pts = input('Which was IM2POINTS run for?:');
end

if ~isfield(options,'cams_cal')
    options.cams_cal = input('Which was GOCAL run for?:');
end
%if the global frame is 1 then the 
%load([options.path,filesep,'CalVal10_06102016.mat'],'cam')

load([options.path,filesep,'..',filesep,'Calibration_run',filesep,'Extrinsic',filesep,'Svoboda',filesep,'Ce.dat']);
load([options.path,filesep,'..',filesep,'Calibration_run',filesep,'Extrinsic',filesep,'Svoboda',filesep,'Re.dat']);
%load([options.path,filesep,'..',filesep,'Calibration_run',filesep,'Extrinsic',filesep,'Svoboda',filesep,'points.dat']);
for cc = 1:ncam
    index_im2pts = find(options.cams_im2pts == cams(cc));
    index_cal  = find(options.cams_cal == cams(cc));
    cam_num = cams(cc);
    %load and store extrinsics
    t   = -Ce(:,index_cal);
    Rot = Re(3*index_cal-2:3*index_cal,:)';
    if ~isempty(Rot) && ~isempty(t)
        camstruct(cam_num).H = [Rot,t;0,0,0,1];
    end
    %Load and store distortion parameters
%     [K,kc] = readradfile_mb([options.path,filesep,'..',filesep,'Calibration_run',filesep,'Intrinsic',filesep,'CalTech',filesep,'Cam',num2str(cam_num),filesep,'int_cam',num2str(cam_num),'.mat']);
%     camstruct(cam_num).K_dist = K; %note that this is a different K than the one used for triangulation
%     camstruct(cam_num).kc = kc;
    
    %Open the .cal file
    fid  = fopen([options.path,filesep, '..',filesep,'Calibration_run',filesep,'Extrinsic',filesep,'Svoboda', filesep,'camera',num2str(cam_num),'.cal']);
    if fid >0
        data = textscan(fid,'%s%s%f');      %Read the data
        %A hard lesson learned: K matrix from cal file must be negated
        camstruct(cam_num).K = -[data{3}(13:15)';data{3}(16:18)';data{3}(19:21)'];
    end
    
    %Open the .rad file
    fid  = fopen([options.path,filesep, '..',filesep,'Calibration_run',filesep,'Extrinsic',filesep,'Svoboda', filesep,'Cam',num2str(cam_num),'.rad']);
    if fid >0
        data = textscan(fid,'%s%s%f');      %Read the data
        %A hard lesson learned: K matrix from cal file must be negated
        camstruct(cam_num).K_dist = [data{3}(1:3)';data{3}(4:6)';data{3}(7:9)'];
        camstruct(cam_num).kc_dist = data{3}(10:14)';
        camstruct(cam_num).fc_dist = [data{3}(1),data{3}(5)];
        camstruct(cam_num).cc_dist = [data{3}(3);data{3}(6)];
        camstruct(cam_num).alpha_c_dist = data{3}(2);
    end
    %camstruct(cam_num).idin = cam(index).idin;
    %npts = size(points,2);
    %all_pt_nums = 1:npts;
    %outliers = setdiff(all_pt_nums,cam(index).idin);
    %camstruct(cam_num).pts_dist  = points(3*index_im2pts-2:3*index_im2pts-1,:);
    %camstruct(cam_num).pts_dist(:,outliers)  = NaN;

    fclose('all');
end
