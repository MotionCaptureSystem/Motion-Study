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

% Last Modified by GUIDE v2.5 29-Oct-2015 15:08:32

% Begin initialization code - DO NOT EDIT
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

handles.Cam(1).feat = {[]};
handles.Cam(1).start_frame = [];
handles.Cam(1).end_frame = [];
handles.Cam(1).H = [];
handles.Cam(1).K = [];
handles.Cam(1).kc = [];
handles.Cam(1).bw_mask = [];

%store the working directory.
handles.options.working = pwd;

%store defualt data directory 
handles.options.path = ['C:',filesep];
handles.options.cams = [];

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
varargout{1} = handles.output;


% --------------------------------------------------------------------
function file_Callback(hObject, eventdata, handles)
% hObject    handle to file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function status_report_Callback(hObject, eventdata, handles)
% hObject    handle to status_report (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function im_proc_Callback(hObject, eventdata, handles)
% hObject    handle to im_proc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function feat_ident_Callback(hObject, eventdata, handles)
% hObject    handle to feat_ident (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function calibration_Callback(hObject, eventdata, handles)
% hObject    handle to calibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function traj_est_Callback(hObject, eventdata, handles)
% hObject    handle to traj_est (see GCBO)
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
handles.Cam = BatIsolation_PointRecognition_local_mb(handles.Cam, handles.options);
guidata(hObject, handles);


% --------------------------------------------------------------------
function surf_ident_Callback(hObject, eventdata, handles)
% hObject    handle to surf_ident (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


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
function tracking_Callback(hObject, eventdata, handles)
% hObject    handle to tracking (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function intrinsic_caltech_Callback(hObject, eventdata, handles)
% hObject    handle to intrinsic_caltech (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --------------------------------------------------------------------
function extrinsic_svoboda_Callback(hObject, eventdata, handles)
% hObject    handle to extrinsic_svoboda (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


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
        load([handles.options.path,filesep,'CamStruct.mat'])
        handles.Cam = Cam; %all data is contained in the Cam Structure
        for cc = 1:length(handles.Cam) %for each camera, load the data
            if isempty(handles.Cam(cc).start_frame) 
            	continue;  %if start_frame is an empty matrix there was no data imported for that camera
            end
            handles.options.cams = [handles.options.cams, cc];
        end
    end
end

guidata(hObject, handles);


% --------------------------------------------------------------------
function bw_mask_Callback(hObject, eventdata, handles)
% hObject    handle to bw_mask (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.Cam = Get_bwbat(handles.Cam,handles.options);
guidata(hObject, handles);


% --------------------------------------------------------------------
function save_Callback(hObject, eventdata, handles)
% hObject    handle to save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
for cc = 1:length(handles.Cam)
    handles.Cam(cc).frames = [];
    handles.Cam(cc).mask   = [];
end
save([handles.options.path,filesep,'CamStruct.mat'], '-struct', 'handles','Cam','-v7.3')
fprintf('The session has been saved.\n')


% --------------------------------------------------------------------
function import_Callback(hObject, eventdata, handles)
% hObject    handle to import (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cam_folders = dir([handles.options.path,filesep,'Cam*.']);
ncam = length(cam_folders);

cams = [];
for cc = 1:ncam
    cam_num = str2double(cam_folders(cc).name(4:6));
    if ~isempty(dir([handles.options.path,filesep,cam_folders(cc).name,filesep,'*.jpg']))
    cams = [cams, cam_num];
    end
end

if isempty(cams);
    fprintf('No images found in this project.  Delace images before proceeeding.\n')
else
    fprintf('Cam numbers '); 
    for cc = 1:length(cams)-1; fprintf('%i, ',cc); end; fprintf('and %i ',cc+1);
    fprintf('have been delaced.\n');
    user_spec = input('Which cams would you like to use? Enter = all:');
    if isempty(user_spec)
        handles.options.cams = cams;
    else
        handles.options.cams = user_spec;
    end
end
    
for cc = handles.options.cams
    img_list = dir([handles.options.path,filesep,cam_folders(cc).name,filesep,'*.jpg']);
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
    