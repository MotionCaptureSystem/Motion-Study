function varargout = MotionStudy(varargin)
% MOTIONSTUDY MATLAB code for MotionStudy.fig
%      MOTIONSTUDY, by itself, creates a new MOTIONSTUDY or raises the existing
%      singleton*.
%
%      H = MOTIONSTUDY returns the handle to a new MOTIONSTUDY or the handle to
%      the existing singleton*.
%
%      MOTIONSTUDY('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MOTIONSTUDY.M with the given input arguments.
%
%      MOTIONSTUDY('Property','Value',...) creates a new MOTIONSTUDY or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MotionStudy_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MotionStudy_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MotionStudy

% Last Modified by GUIDE v2.5 24-Oct-2016 14:44:23

% Begin initialization code - DO NOT EDIT
addpath('Calibration', 'Common', 'FeatIdent','ImageProc','MotionEst','Results','PtsInfo','CFDPrep')
addpath(['.',filesep,'MotionEst',filesep,'Init'],...
        ['.',filesep,'MotionEst',filesep,'Models'],...
        ['.',filesep,'MotionEst',filesep,'Stereo'],...
        ['.',filesep,'MotionEst',filesep,'TrajEst'],...
        ['.',filesep,'MotionEst',filesep,'Manifolds'],...
        ['.',filesep,'MotionEst',filesep,'Manifolds',filesep,'GPDM'],...
        ['.',filesep,'MotionEst',filesep,'Tracking'])

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MotionStudy_OpeningFcn, ...
                   'gui_OutputFcn',  @MotionStudy_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
               
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before MotionStudy is made visible.
function MotionStudy_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MotionStudy (see VARARGIN)

% Choose default command line output for MotionStudy
handles.output = hObject;

%initialize empty structure values
handles.Cam(1).feat = {[]};
handles.Cam(1).start_frame = [];
handles.Cam(1).end_frame = [];
handles.Cam(1).H = [];
handles.Cam(1).K = [];
handles.Cam(1).kc = [];
handles.Cam(1).b_box = [];
handles.Cam(1).frame = [];

%store the working directory.
handles.options.working = pwd;
handles.options.fs_c = 119.88;
handles.options.fs_a = 48000;
handles.options.ucs_size = 0.05;
%add subdirectories

%store defualt data directory 
handles.options.path = ['.',filesep];
handles.options.cams = [];

%store the current timestep and camera number
handles.options.current_cam=[];
handles.options.current_timestep=[];

%store the start time and end time
fig1_child = get(handles.figure1,'Children');
handles.options.bar_starttime=get(fig1_child(2),'String');
%handles.options.bar_endtime=get(fig1_child(1),'String');

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes MotionStudy wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = MotionStudy_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
rmpath('Calibration', 'Common', 'FeatIdent','ImageProc','MotionEst','Results')
varargout{1} = handles.output;


% --------------------------------------------------------------------
function file_menu_Callback(hObject, eventdata, handles)
% hObject    handle to file_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
test = 5;
guidata(hObject,handles)

% --------------------------------------------------------------------
function tracking_Callback(hObject, eventdata, handles)
% hObject    handle to file_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function status_report_Callback(hObject, eventdata, handles)
% hObject    handle to status_report (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function im_proc_menu_Callback(hObject, eventdata, handles)
% hObject    handle to im_proc_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function feat_ident_menu_Callback(~, eventdata, handles)
% hObject    handle to feat_ident_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function calibration_menu_Callback(hObject, eventdata, handles)
% hObject    handle to calibration_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function motion_est_menu_Callback(hObject, eventdata, handles)
% hObject    handle to motion_est_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function motion_ident_Callback(hObject, eventdata, handles)
% hObject    handle to motion_ident (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function delace_im_Callback(hObject, eventdata, handles)
% hObject    handle to delace_im (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cam_fold = dir([handles.options.path,filesep,'Cam*.']);

cams = zeros(1,length(cam_fold));

fprintf('The following cams are available for delacing:\n')
for cc = 1:length(cam_fold)
    cams(cc) = str2double(cam_fold(cc).name(4:6));
    fprintf('%d, ',cams(cc))
end
fprintf('\b\b\n')
cams2delace = input('Which Cams would you like to delace? [enter = all]:');
if isempty(cams2delace)
    cams2delace = cams;
end

frames2delace = input('Which Frames would you like to delace? [enter = all]:');

for cc = 1:length(cams2delace)
    fprintf('Delacing Camera %d Video...\n',cams2delace(cc))
    cam_num = num2str(cams2delace(cc));
    while length(cam_num)<3
        cam_num = ['0',cam_num];
    end
    vid_slice([handles.options.path,filesep,'Cam',cam_num],['cam',cam_num,'.MP4'],'png', frames2delace)
end


function vid_slice(dirname,filename, im_type, frames)
%VID_SLICE          -Slices the video in FILENAME into individual images of
% filetype TYPE.

Video = VideoReader([dirname,filesep,filename]);  %Read the video file
nframe = Video.NumberOfFrames;             %Determine the number of frames
if isempty(frames)
   frames = 2:nframe-1; 
end

%The delacer has a bug where if the first or last frame is delaced there is
%a whole frame of delay.  This bug is sporadic.  Therefore, if the first or
%last frame has been specified, chop them out of the selection.
if frames(1) == 1
    frames(1) = [];
end
if frames(end) == nframe
    frames(end) = [];
end
%kk = 0;
%wbhandle = waitbar(0,['Delacing ',filename,'...']);
for ff = frames
    %kk = kk+10;
    frame=read(Video,ff);                       %read the frame
    name=strcat(dirname,filesep,num2str(ff),'.', im_type);
    imwrite(frame,name);
    if ff == 1
        imwrite(frame,[dirname,filesep,'bkgnd.png'])
    end
    %waitbar(ii/nframe,wbhandle,['Delacing ',filename,'...',sprintf('%3.1f',ii*100/nframe),'%']);
end
%delete(wbhandle);


% --------------------------------------------------------------------
function color_conversion_Callback(hObject, eventdata, handles)
% hObject    handle to color_conversion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function im_enhance_Callback(hObject, eventdata, handles)
% hObject    handle to im_enhance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function back_sub_Callback(hObject, eventdata, handles)
% hObject    handle to back_sub (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Yousi_Ident_Callback(hObject, eventdata, handles)
% hObject    handle to Yousi_Ident (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Cam = log_gauss_var_thresh_feats(handles.Cam, handles.options);
guidata(hObject, handles);


% --------------------------------------------------------------------
function surf_ident_Callback(hObject, eventdata, handles)
% hObject    handle to surf_ident (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

scale = 3;
for cc = handles.options.cams
    if ~isfield(handles.Cam(cc), 'features')
        handles.Cam(cc).features = []; 
    end
    if ~isempty(handles.Cam(cc).features)
        y = input(sprintf('Features for Camera %d have already been computed.  Do you want to recompute [Y or N]?',cc),'s');
        if strcmpi(y,'Y')
            
        else
            continue
        end
    end
    images = dir([handles.options.path,filesep,'Cam', num2str(cc), filesep, '*.png']);
    for ii = 1:length(images)-1
        if ~isnan(handles.Cam(cc).b_box(ii,:)) & ~handles.Cam(cc).b_box(ii,:)==0
        %get previously computed region of interest
        xc = handles.Cam(cc).b_box(ii,1);
        yc = handles.Cam(cc).b_box(ii,2);
        xw = handles.Cam(cc).b_box(ii,3);
        yw = handles.Cam(cc).b_box(ii,4);

        %read the image
        img = imread( [handles.options.path,filesep,'Cam', num2str(cc), filesep, images(ii).name]);
        if xc+xw > size(img,2)
            xw = size(img,2)-xc;
        end
        if yc+yw > size(img,1)
            yw = size(img,1)-yc;
        end
        %contrast adjust the image
        %imgFilt=imadjust(img(yc:yc+yw,xc:xc+xw,:),[0 0 0; 0.1 0.1 0.1],[]);
        %convert the color image to grayscale
        imgFilt=rgb2gray(img(yc:yc+yw,xc:xc+xw,:));
        %enlarge the image
        imgFilt=imresize(imgFilt,scale);
        %filter the image
        G=fspecial('gaussian',[39 39],0.1);
        imgFilt = imfilter(imgFilt,G,'same');
        %find the surf features
        points=detectSURFFeatures(imgFilt,'MetricThreshold',1000,'NumScaleLevels',4,'NumOctaves', 5);
        if ~isempty(points)
            index = find(points.SignOfLaplacian<0);
            feat =  points.selectStrongest(length(points));
            handles.Cam(cc).features{ii} = feat(index,:);
            handles.Cam(cc).features{ii}.Location(:,1) = handles.Cam(cc).features{ii}.Location(:,1)/scale + xc;
            handles.Cam(cc).features{ii}.Location(:,2) = handles.Cam(cc).features{ii}.Location(:,2)/scale + yc;
        else
            handles.Cam(cc).features{ii}.Location(:,1) = NaN;
            handles.Cam(cc).features{ii}.Location(:,2) = NaN;
        end
        figure(1)
        imshow(imgFilt)
        hold on
        plot(3*(handles.Cam(cc).features{ii}.Location(:,1)-xc), 3*(handles.Cam(cc).features{ii}.Location(:,2)-yc),'+r')
        hold off
        %pause
        guidata(hObject, handles);
        end
    end
end


% --------------------------------------------------------------------
function integrated_workflows_Callback(hObject, eventdata, handles)
% hObject    handle to integrated_workflows (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function intrinsic_cal_Callback(hObject, eventdata, handles)
% hObject    handle to intrinsic_cal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function extrinsic_cal_Callback(hObject, eventdata, handles)
% hObject    handle to extrinsic_cal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function point_pick_Callback(hObject, eventdata, handles)
% hObject    handle to point_pick (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%handles.Cam = track_points_im(handles.Cam,handles.options.path);
setappdata(0,'MStudyHands', handles);
try 
    PointPropagatorV1
    uiwait 
catch ME, 
    MStudyHandles = getappdata(0,'MStudyHands');
end

MStudyHandles = getappdata(0,'MStudyHands');
handles.Cam = MStudyHandles.Cam;
guidata(hObject, handles);



% --------------------------------------------------------------------
function intrinsic_caltech_Callback(hObject, eventdata, handles)
% hObject    handle to intrinsic_caltech (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cams = input('Import INTRINSIC parameters for which Cameras?:');
[handles.Cam] = load_caltech_intrinsic(handles.Cam,handles.options,cams);
guidata(hObject,handles);



% --------------------------------------------------------------------
function extrinsic_svoboda_Callback(hObject, eventdata, handles)
% hObject    handle to extrinsic_svoboda (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%import all available calibration_menu parameters
[handles.options, handles.Cam] = load_svoboda_cal5(handles.Cam, handles.options);
guidata(hObject,handles);


% --------------------------------------------------------------------
function intrinsic_april_Callback(hObject, eventdata, handles)
% hObject    handle to intrinsic_april (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function intrinsic_matlab_Callback(hObject, eventdata, handles)
% hObject    handle to intrinsic_matlab (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function filepath_Callback(hObject, eventdata, handles)
% hObject    handle to filepath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.options.path = uigetdir(handles.options.path);

if exist([handles.options.path,filesep,'CamStruct.mat'],'file') %If there is a data file, should it be loaded?
    l = input('A previous data file exists. Do you want to load it? [y/n]','s');
    if strcmp(l,'y') %If the user specifies yes, load the data
        handles = rmfield(handles,'Cam');   %Clear the initialized Cam structure
        fprintf('Loading CamStruct.mat ... \n')
        load([handles.options.path,filesep,'CamStruct.mat'])
        if exist('cams', 'var')   %if the files is saved in the new format, store as follows
            handles.Cam(cams(1):cams(2)) = Cam;
        else %if the file is saved in the old format, store as follows
            handles.Cam = Cam; %all data is contained in the Cam Structure
        end
        %if trajectory estimation has been performed, load that too
        if exist([handles.options.path,filesep,'EstStruct.mat'],'file')
            fprintf('Loading EstStruct.mat ... \n')
            loaded = load([handles.options.path,filesep,'EstStruct.mat']);
            if isfield(loaded,'options')
                optionsNew = loaded.options;
                names = fieldnames(handles.options);
                for nn = 1:length(names)
                    if ~isfield(optionsNew,names{nn})
                        optionsNew.(names{nn}) = eval(['handles.options.',names{nn}]);
                    end
                end
                optionsNew.path = handles.options.path;
                handles.options = optionsNew;
            end
            if isfield(loaded,'kinc')
                EstStruct.kinc = loaded.kinc;
            end
            if isfield(loaded, 'ukf')
                EstStruct.ukf = loaded.ukf;
            end
            handles.EstStruct = EstStruct;
        end
        
        %if there is stereo triangluation data saved, load that too
        if exist([handles.options.path,filesep,'StereoStruct.mat'],'file')
            fprintf('Loading StereoStruct.mat ... \n')
            load([handles.options.path,filesep,'StereoStruct.mat'])
            handles.Stereo = Stereo;
        end
        handles.options.cams = [];       
        for cc = 1:length(handles.Cam) %for each camera, load the data
            if isempty(handles.Cam(cc).start_frame) 
            	continue;  %if start_frame is an empty matrix there was no data imported for that camera
            end
            handles.options.cams = [handles.options.cams, cc];
        end
        if isfield(Cam,'pts')
        setappdata(0,'Cam', Cam);
        %display the ToolBar of the figure
        setappdata(0,'handles',handles)
        set(handles.figure1,'ToolBar','figure')
        if isfield(Cam,'sync_del')
%            BarGraph(handles);
        end
        end
    end
end

guidata(hObject, handles);


% --------------------------------------------------------------------
function bw_mask_Callback(hObject, eventdata, handles)
% hObject    handle to bw_mask (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.Cam = get_bwbat(handles.Cam,handles.options);
guidata(hObject, handles);


% --------------------------------------------------------------------
function save_Callback(hObject, eventdata, handles)
% hObject    handle to save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
for cc = 1:length(handles.Cam)
    handles.Cam(cc).frames = [];
end
fprintf('Saving data ... do not close MATLAB or MOTIONSTUDY ... \n')
cams = which_cams(handles.Cam);
cams = [cams(1), cams(end)];
Cam = handles.Cam(cams(1):cams(2));
save([handles.options.path,filesep,'CamStruct.mat'], 'Cam', 'cams')
if isfield(handles, 'Stereo')
    Stereo = handles.Stereo;
    save([handles.options.path,filesep,'StereoStruct.mat'],'Stereo')
end
if isfield(handles, 'EstStruct')
    %handles.EstStruct.options = handles.options;
    ukf = handles.EstStruct.ukf;
    options = handles.options;
    if isfield(options,'est')
        if isfield(options.est,'Rt_handle')
            options.est = rmfield(options.est, 'Rt_handle');
        end
        if isfield(options,'msmt_model')
            options.est = rmfield(options.est, 'msmt_model');
        end
        if isfield(options,'state_update_model')
            options.est = rmfield(options.est, 'state_update_model');
        end
    end
    if isfield(handles.EstStruct, 'kinc')
        kinc = handles.EstStruct.kinc;
        save([handles.options.path,filesep,'EstStruct.mat'],'options','kinc','ukf')
    else
        save([handles.options.path,filesep,'EstStruct.mat'],'options','ukf')
    end
    if isfield(handles.EstStruct, 'n_correct')
        n_correct = handles.EstStruct.n_correct;
        if isfield(handles.EstStruct, 'z_gg_auto')
            z_gg_auto = handles.EstStruct.z_gg_auto;
            save([handles.options.path,filesep,'Correct_Corresp.mat'],'n_correct','z_gg_auto')
        else
            save([handles.options.path,filesep,'Correct_Corresp.mat'],'n_correct')    
        end
    end
end
alarmS = load('chirp'); sound(alarmS.y,alarmS.Fs);
%Play a sound when save finished.
fprintf('The session has been saved.\n')


% --------------------------------------------------------------------
function import_Callback(hObject, eventdata, handles)
% hObject    handle to import (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cam_folders = dir([handles.options.path,filesep,'Cam*.']);
ncam = length(cam_folders);

cams = [];
nonempty_folders = [];
for cc = 1:ncam
    cam_num = str2double(cam_folders(cc).name(4:6));
    if ~isempty(dir([handles.options.path,filesep,cam_folders(cc).name,filesep,'*.png']))
    cams = [cams, cam_num];
    nonempty_folders=[nonempty_folders,cam_folders(cc)];
    end
end

if isempty(cams);
    fprintf('No images found in this project.  Delace images before proceeeding.\n')
else
    fprintf('The following cameras have delaced images: \n'); 
    for cc = cams; fprintf('%i\t',cc); end; fprintf('\n');
    user_spec = input('Which cams would you like to use? Enter = all:');
    if isempty(user_spec)
        handles.options.cams = cams;
    else
        handles.options.cams = user_spec;
    end
end
    
for cc = handles.options.cams
    img_list = dir([handles.options.path,filesep,nonempty_folders(cc==cams).name,filesep,'*.png']);
    %img_list = dir([handles.options.path,filesep,cam_folders(cc).name,filesep,'*.png']);
    %Wrong correlation when: (causing start & stop frame number wrong)
    %Import all cams, meeting empty folders(302 is empty, 301->301 & 303->302 304->303)
    %Import cams which don't begin with 301([302:305]->[301:304])
    timesteps = [];
    for pp = 1:length(img_list)
        pic_num = sscanf(img_list(pp).name,'%f');
        if ~isnan(pic_num)
            timesteps = [timesteps, pic_num];
        end
    end
    handles.Cam(cc).start_frame = min(timesteps);
    handles.Cam(cc).end_frame   = max(timesteps);
end
guidata(hObject, handles);
    


% --------------------------------------------------------------------
function bound_box_Callback(hObject, eventdata, handles)
% hObject    handle to bound_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

for cc = handles.options.cams
    if isempty(handles.Cam(cc).b_box)
        handles.Cam = find_bat(num2str(cc),handles.Cam,handles.options);
        guidata(hObject,handles)
    else
        fprintf('Bounding boxes for camera %d already computed ... \n',cc)
    end
end


% --------------------------------------------------------------------
function pix_space_track_Callback(hObject, eventdata, handles)
% hObject    handle to pix_space_track (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.Cam = track_points_im(handles.Cam,handles.options.path);
guidata(hObject,handles);


% --------------------------------------------------------------------
function audio_sync_Callback(hObject, eventdata, handles)
% hObject    handle to audio_sync (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% determine the sync delay by reading the audio track of the video file_menu.
y = input('Were the cameras hardware synced? [y/n]','s');
if strcmpi(y,'y')
    for ii = handles.options.cams
        handles.Cam(ii).sync_del = 0;
        handles.Cam(ii).pts_sync = handles.Cam(ii).pts_rect;
    end
    handles.options.fs_c = input('What sampling rate was used?:');
else
    if isfield(handles.Cam,'sync_del')
        y = input('Sync Data Already Exists.  Do you want to recompute the delays?:','s');
        if strcmp(y, 'y')
            [handles.options, handles.Cam] = audiosync(handles.Cam, handles.options);
        end
    else
    [handles.options, handles.Cam] = audiosync(handles.Cam, handles.options);
    [handles.Cam] = subframe_sync(handles.Cam, handles.options);
    end
end

% perform subframe synchronization of cameras (linear interp)
guidata(hObject,handles);


% --------------------------------------------------------------------
function bundle_adjust_Callback(hObject, eventdata, handles)
% hObject    handle to bundle_adjust (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load_bund = input('Would you like to load a bundle adjustment file?','s');

if strcmpi(load_bund, 'y')
    handles.options.ba = 1;
end

handles.Cam = bundle_adjust_pixel2(handles.Cam, handles.options);
guidata(hObject, handles);


% --------------------------------------------------------------------
function stereo_triangulation_Callback(hObject, eventdata, handles)
% hObject    handle to stereo_triangulation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.options.stereo.cams   = input('Which CAMERAS should be used for stereo triangulation:');
handles.options.stereo.pts    = input('Which POINTS should be used for stereo triangulation:');
handles.options.stereo.tstart = input('What START time should be used for stereo triangulation:');
handles.options.stereo.tstop  = input('What STOP time should be used for stereo triangulation:');
handles.options.stereo.dt     = input('Plot every ?TH frame:');
%handles.options.plot.linestyle1 = {'.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g','+-r','+-b','+-g', '+-m','+-k','+-c','+--r','+--b','+--g'};
handles.options.plot.colors     = hsv(length(handles.options.stereo.pts));
%handles.options.plot.linestyle3 = {'o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g','.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g'};

handles.Stereo = StereoTriangulation_svob(handles.Cam, handles.options);
guidata(hObject, handles);

% --------------------------------------------------------------------
function rectify_Callback(hObject, eventdata, handles)
% hObject    handle to rectify (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cams = input('In which cams should points be rectified?:');
%timesteps = handles.:tstop;

for c = cams
    fprintf('Rectifying Points from Camera %d ... \n',c)
    pt = 0;
    %load([handles.options.path,filesep,'..',filesep,'Calibration_run',filesep,'Intrinsic',filesep,'CalTech',filesep,'Cam',num2str(c),filesep,'int_cam',num2str(c),'.mat'], 'KK','kc','alpha_c','fc','cc');
   
    handles.Cam(c).pts_rect = NaN*zeros(size(handles.Cam(c).pts));
    pts = 1:size(handles.Cam(c).pts,3);
    for pp = pts
        pt = pt+1;
        for kk = 1:size(handles.Cam(c).pts,2)
            %if the point is occluded the value will be NaN
            point = handles.Cam(c).pts(:,kk,pp);
            point_ud = rm_distortion(point, handles.Cam(c).K_dist, handles.Cam(c).fc_dist,...
                       handles.Cam(c).cc_dist, handles.Cam(c).alpha_c_dist, handles.Cam(c).kc_dist);
            handles.Cam(c).pts_rect(:,kk,pts(pt)) = point_ud;
        end
    end
end

guidata(hObject,handles);

% --------------------------------------------------------------------
function rm_points_Callback(hObject, eventdata, handles)
% hObject    handle to rm_points (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cams = input('Enter the CAMERAS which you would like to remove points from: ');
pts = input('Enter the POINT numbers you would like to visualize:');
for cam = cams
    figure
    hold on
    for pp = pts
        %plot the distorted features
        if size(handles.Cam(cam).pts,3)>=pp && ~isempty(handles.Cam(cam).pts)
        plot(handles.Cam(cam).pts(1,:,pp)',handles.Cam(cam).pts(2,:,pp)','+r');
        plot(handles.Cam(cam).pts(1,:,pp)',handles.Cam(cam).pts(2,:,pp)','-b');
        %plot the undistorted features
        plot(handles.Cam(cam).pts_rect(1,:,pp)',handles.Cam(cam).pts_rect(2,:,pp)','og');
        plot(handles.Cam(cam).pts_rect(1,:,pp)',handles.Cam(cam).pts_rect(2,:,pp)','-k');    
        %plot the synchronized featured
        plot(handles.Cam(cam).pts_sync(1,:,pp)',handles.Cam(cam).pts_sync(2,:,pp)','^b');
        plot(handles.Cam(cam).pts_sync(1,:,pp)',handles.Cam(cam).pts_sync(2,:,pp)','-m');  
        end
    end
    title(['Camera ',num2str(cam)]);
end

% --------------------------------------------------------------------
function scale_extrinsics_Callback(hObject, eventdata, handles)
% hObject    handle to scale_extrinsics (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
scale = input('By what factor would you like to scale extrinsics?:');
handles.options.ucs_size = handles.options.ucs_size*scale;
for cc = 1:length(handles.Cam)
    if ~isempty(handles.Cam(cc).H)
        handles.Cam(cc).H(1:3,4) = handles.Cam(cc).H(1:3,4)*scale;
    end
end
guidata(hObject,handles)


% --------------------------------------------------------------------
function results_menu_Callback(hObject, eventdata, handles)
% hObject    handle to results_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function plotting_Callback(hObject, eventdata, handles)
% hObject    handle to plotting (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --------------------------------------------------------------------
function traject_est_Callback(hObject, eventdata, handles)
% hObject    handle to traject_est (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function est_states_plot_Callback(hObject, eventdata, handles)
% hObject    handle to est_states_plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if strcmp(handles.options.est.type,'joint')
    Plot_Script_JS(handles.Cam, handles.EstStruct, handles.options)
elseif strcmp(handles.options.est.type,'point')
    PlotScript(handles.Cam, handles.EstStruct, handles.options)
end

% --------------------------------------------------------------------
function reproj_plots_Callback(hObject, eventdata, handles)
% hObject    handle to reproj_plots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
reproj_error(handles.Cam,handles.EstStruct,handles.options);

% --------------------------------------------------------------------
function reproj_error_plots_Callback(hObject, eventdata, handles)
% hObject    handle to reproj_error_plots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function merge_proj_Callback(hObject, eventdata, handles)
% hObject    handle to merge_proj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%prompt the user to select which file should be merged.
[filename, pathname, ~] = uigetfile('*.mat','Select Project to Merge',handles.options.path);
import_struct = load([pathname,filename]);
% determine the cameras that are being imported
cams = [import_struct.cams(1):import_struct.cams(2)];
ncam = length(cams);

%Determine which Fields are in the current project
Cam = handles.Cam;
fields_orig = fieldnames(Cam); 
fields_imp = fieldnames(import_struct.Cam); 
%%IMPORT MISSING FIELDS OF DATA
%for each camera in the current project, determine if there are missing
%data that exist in the project being imported.
fprintf('Finding data in the imported structure which is not contained in this project...\n')
% %determine which fields in this project are empty
% orig_fields = structfun(@isempty, Cam(cams(c)));
% %determine which fields in the imported structure are full
% import_fields = ~structfun(@isempty, import_struct.Cam(c));

%check each field in import struct, and make sure it is in original 
for ii = 1:length(fields_imp)                       
    if ~any(strcmp(fields_imp{ii},fields_orig))     %if this field is in the imported struct and not in original 
        Cam(1).(fields_imp{ii}) = [];               %add place holder
    end
    for kk = 1:ncam            %for all imported cameras
        if isempty(Cam(cams(kk)).(fields_imp{ii}))  %if that field is empty 
            %import the data into this project
            Cam(cams(kk)).(fields_imp{ii}) = import_struct.Cam(kk).(fields_imp{ii});
        end
    end
end

% for c = 1:ncam
%     %get the camera number 
%     fprintf('Importing From Cam %i...\n',cams(c))
% 
%     for ff = 1:length(fields_orig);
%         if iscell(Cam(cams(c)).(fields_orig{ff}))
%             if isempty(Cam(cams(c)).(fields_orig{ff}){1})
%                 orig_fields(ff) = 0;
%             end
%         end
%     end
%     for ff = 1:length(fields_orig);
%         if iscell(Cam(cams(c)).(fields_orig{ff}))
%             if isempty(Cam(cams(c)).(fields_orig{ff}){1})
%                 import_fields(ff) = 0;
%             end
%         end
%     end
%     %determine where the imported project has data and the current project
%     %doesn't
%     overlap = orig_fields & import_fields;
%     %for each field in the list, import what is missing in the current
%     %structure.
%     for ff = 1:length(fields_orig);
%         if overlap(ff)
%            Cam(cams(c)).(fields_orig{ff}) = import_struct.Cam(c).(fields_orig{ff});
%         end
%     end   
% end

%%IMPORT MISSING POINTS
for c = 1:ncam
    import_fields = ~structfun(@isempty, import_struct.Cam(c));
    for ff = 1:length(fields_orig);
        if iscell(Cam(cams(c)).(fields_orig{ff}))
            if isempty(Cam(cams(c)).(fields_orig{ff}){1})
                import_fields(ff) = 0;
            end
        end
    end
    if sum(import_fields)==0
       continue 
    end
%     figure
%     hold on 
%     for pp = 1:size(import_struct.Cam(c).pts,3)
%     plot(import_struct.Cam(c).pts(1,:,pp)',import_struct.Cam(c).pts(2,:,pp)','+r')
%     end
%     for pp = 1:size(Cam(cams(c)).pts,3)
%     plot(Cam(cams(c)).pts(1,:,pp)',Cam(cams(c)).pts(2,:,pp)','+g')
%     end
    fprintf('Merging point data from camera %d...\n',c)
    %determine difference in start frames
    dt = Cam(cams(c)).start_frame - import_struct.Cam(c).start_frame;
    fprintf('Start frame difference: %d.\n',dt)
    if dt > 0  %if difference is negative, pad the original point matrix
        Cam(cams(c)).pts = [NaN*zeros(2,abs(dt),size(Cam(cams(c)).pts,3)),Cam(cams(c)).pts];
        Cam(cams(c)).start_frame = import_struct.Cam(c).start_frame;
    elseif dt < 0 %if the difference is positive, pad the imported point matrix
        import_struct.Cam(c).pts = [NaN*zeros(2,dt,size(import_struct.Cam(c).pts,3)),import_struct.Cam(c).pts];
        import_struct.Cam(c).start_frame = Cam(cams(c)).start_frame;
    end
    
    %determine if the matricies of point data are the same length
    l_orig = size(Cam(cams(c)).pts,2);
    l_import = size(import_struct.Cam(c).pts,2);
    dl = l_orig - l_import;
    fprintf('Matrix length difference: %d.\n',dl)
    if dl<0 %if difference is negative, pad the original point matrix
        Cam(cams(c)).pts = [Cam(cams(c)).pts, NaN*zeros(2,abs(dl),size(Cam(cams(c)).pts,3))];
    elseif dl>0 %if the difference is positive, pad the imported point matrix
        import_struct.Cam(c).pts = [import_struct.Cam(c).pts, NaN*zeros(2,dl,size(import_struct.Cam(c).pts,3))];
    end
    
    %determine if the matricies of point data contain the same number of points
    npts_orig   = size(Cam(cams(c)).pts,3);
    npts_import = size(import_struct.Cam(c).pts,3);
    dp = npts_orig - npts_import;
    fprintf('Point number difference: %d.\n',dp)
    if dp <0 %if difference is negative, pad the original point matrix
        Cam(cams(c)).pts(:,:,npts_orig+1:npts_orig+abs(dp)) = NaN*zeros(2,size(Cam(cams(c)).pts,2),abs(dp));
    elseif dp>0 %if the difference is positive, pad the imported point matrix
        import_struct.Cam(c).pts(:,:,npts_import+1:npts_import+abs(dp)) = NaN*zeros(2,size(import_struct.Cam(c).pts,2),abs(dp));
    end
    
    %determine which points were not picked in the original project
    not_exist_pts_orig = isnan(Cam(cams(c)).pts);
    %determine which points were picked in the imported data set
    exist_pts_import = ~isnan(import_struct.Cam(c).pts);
    %determine the overlap in points
    if ~isempty(not_exist_pts_orig) && ~isempty(exist_pts_import)
        overlap_pts = not_exist_pts_orig & exist_pts_import;
        if any(any(any(overlap_pts)))
            fprintf('Points exist in imported project which do not exist in current project.\n')
            fprintf('Importing points .... \n')
            %copy the points from the imported dataset to the current dataset
            Cam(cams(c)).pts(overlap_pts) = import_struct.Cam(c).pts(overlap_pts);
        end
        
    end
%     for pp = 1:size(Cam(cams(c)).pts,3)
%         plot(Cam(cams(c)).pts(1,:,pp)',Cam(cams(c)).pts(2,:,pp)','ob')
%     end
end

handles.Cam = Cam;
guidata(hObject, handles)

% --------------------------------------------------------------------
function est_options_Callback(hObject, eventdata, handles)
% hObject    handle to est_options (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[init_file, init_path, ~]       = uigetfile(['.',filesep,'MotionEst',filesep,'Init'],'Choose your initialization file:');
[handles.Cam, handles.options]  = feval([init_file(1:end-2)], handles.Cam, handles.options);

guidata(hObject, handles);

% --------------------------------------------------------------------
function estimator_Callback(hObject, eventdata, handles)
% hObject    handle to estimator (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[handles.EstStruct, options] = traj_estimation(handles.Cam(handles.options.est.cams), handles.options);

guidata(hObject, handles);


% --------------------------------------------------------------------
function rotate_extrinsics_Callback(hObject, eventdata, handles)
% hObject    handle to rotate_extrinsics (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
yprs = input('enter the Yaw-Pitch-Roll rotations to transform extrinsics:');
Hmod = YPRTransform(yprs,zeros(size(yprs)));
for cc = 1:length(handles.Cam)
    if ~isempty(handles.Cam(cc).H)
        handles.Cam(cc).H = Hmod*handles.Cam(cc).H;
    end
end
guidata(hObject,handles)

% --------------------------------------------------------------------
function translate_extrinsics_Callback(hObject, eventdata, handles)
% hObject    handle to translate_extrinsics (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
trans = input('enter the Yaw-Pitch-Roll rotations to transform extrinsics:');
Hmod = YPRTransform(zeros(size(trans)),trans);
for cc = 1:length(handles.Cam)
    if ~isempty(handles.Cam(cc).H)
        handles.Cam(cc).H = Hmod*handles.Cam(cc).H;
    end
end
guidata(hObject,handles)




%<<<<<<< HEAD
% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1

function Start_time_Callback(hObject, eventdata, handles)
% hObject    handle to Start_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Start_time as text
%        str2double(get(hObject,'String')) returns contents of Start_time as a double


handles.options.bar_starttime=get(hObject,'String');

guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function Start_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Start_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
handles.options.bar_endtime=get(hObject,'String');
handles.options.bar_tottime=handles.options.bar_endtime-handles.options.bar_starttime+1;
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --------------------------------------------------------------------
function cfd_prep_Callback(hObject, eventdata, handles)
% hObject    handle to cfd_prep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function cfd_reproj_Callback(hObject, eventdata, handles)
% hObject    handle to cfd_reproj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file_name, path_name] = uigetfile({'*.mat','MAT-files (*.mat)'}, 'Choose Smoothed Data File for Reprojection');
load([path_name,filesep,file_name])
reproj_error_cfd(handles.Cam,pts,lookuptab,handles.options);
