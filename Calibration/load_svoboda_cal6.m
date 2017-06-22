function [options, camstruct] = load_svoboda_cal6(camstruct,options)

cams = options.cams;
ncam = length(cams);

if ~isfield(options,'cams_im2pts')
    options.cams_im2pts = input('Which was IM2POINTS run for?:');
end

if ~isfield(options,'cams_cal')
    options.cams_cal = input('Which was GOCAL run for?:');
end

for cc = 1:ncam
    cam_num = cams(cc);
    %load and store extrinsics   
    %Open the .cal file
    fid  = fopen([options.path,filesep, '..',filesep,'Calibration_run',filesep,'Extrinsic',filesep,'Svoboda', filesep,'camera',num2str(cam_num),'.cal']);
    if fid >0
        data = textscan(fid,'%s%s%f');      %Read the data
        %A hard lesson learned: K matrix from cal file must be negated
        camstruct(cam_num).K = [data{3}(13:15)';data{3}(16:18)';data{3}(19:21)'];
    end
    t   = data{3}(10:12);
    Rot = reshape(data{3}(1:9),3,3)';
    if ~isempty(Rot) && ~isempty(t)
        camstruct(cam_num).H = [Rot,t;0,0,0,1];
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
