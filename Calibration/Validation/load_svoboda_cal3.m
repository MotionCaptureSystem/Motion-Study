function camstruct = load_svoboda_cal3(options)

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
load([options.path,filesep,'Run1.mat'],'linear')
%load([options.path,filesep,'batflight_20150724_extrinsic.mat'],'Ws_interp')
load([options.path,filesep,'Run1.mat'],'C');
load([options.path,filesep,'Run1.mat'],'R');
load([options.path,filesep,'Run1.mat'],'cam');
load([options.path,filesep,'Run1.mat'],'inliers');
Ws = linear.Ws;
for cc = 1:ncam
    index = find([cam.camId] == cams(cc));
    cam_num = cams(cc);
    fid = fopen([options.path,filesep,'camera',num2str(cam_num),'.cal']);
    data = textscan(fid,'%s%s%f');
    %t = data{3}(10:12);
    t = -C(:,index);
    %R = [data{3}(1:3)';data{3}(4:6)';data{3}(7:9)'];
    Rot = R(3*index-2:3*index,:)';
    camstruct(cam_num).H = [Rot,t;0,0,0,1];
    %camstruct(cam_num).Pmat = Pe(3*(cc-1)+1:3*cc,:);
    %camstruct(cam_num).Pmat = Pe(3*(cc-1)+1:3*cc,:);
    load([options.path,filesep,'..\..\Intrinsic\CalTech\Cam',num2str(cam_num),filesep,'int_cam',num2str(cam_num),'.mat'],'KK','kc');
    camstruct(cam_num).K = KK;
 %   load(['..\Intrinsic\Cam',num2str(cam_num),filesep,'int_cam',num2str(cam_num),'.mat'],'kc');
     camstruct(cam_num).kc = kc;
 %   camstruct(cam_num).K = [data{3}(13:15)';data{3}(16:18)';data{3}(19:21)'];
%     for ii = 1:9
%         if camstruct(cam_num).K(ii)<0
%             camstruct(cam_num).K(ii) = -camstruct(cam_num).K(ii);
%         end
%     end
    %camstruct(cam_num).idin = cam(cc).idin;
    camstruct(cam_num).pts      = Ws(3*index-2:3*index-1,inliers.idx);
    camstruct(cam_num).pts(1,:) = camstruct(cam_num).pts(1,:)+KK(1,3);
    camstruct(cam_num).pts(2,:) = camstruct(cam_num).pts(2,:)+KK(2,3);
    fclose('all');
end
