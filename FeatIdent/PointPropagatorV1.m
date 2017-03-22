function varargout = PointPropagatorV1(varargin)
% POINTPROPAGATORV1 MATLAB code for PointPropagatorV1.fig
%      POINTPROPAGATORV1, by itself, creates a new POINTPROPAGATORV1 or raises the existing
%      singleton*.
%
%      H = POINTPROPAGATORV1 returns the handle to a new POINTPROPAGATORV1 or the handle to
%      the existing singleton*.
%
%      POINTPROPAGATORV1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in POINTPROPAGATORV1.M with the given input arguments.
%
%      POINTPROPAGATORV1('Property','Value',...) creates a new POINTPROPAGATORV1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PointPropagatorV1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PointPropagatorV1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PointPropagatorV1

% Last Modified by GUIDE v2.5 23-Mar-2016 15:51:05

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PointPropagatorV1_OpeningFcn, ...
                   'gui_OutputFcn',  @PointPropagatorV1_OutputFcn, ...
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


% --- Executes just before PointPropagatorV1 is made visible.
function PointPropagatorV1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PointPropagatorV1 (see VARARGIN)

MStudyHandles = getappdata(0,'MStudyHands');
set(findobj(handles.figure1),'Units','Normalized')

%initialize Cam structure with empty fields
handles.Cam = MStudyHandles.Cam;
for cc = 1:length(handles.Cam)
    if ~isfield(handles.Cam(cc), 'pts')
        handles.Cam(cc).pts = [];
    end
end

%store working directory
handles.working = pwd;
%import handles from MotionStudy GUI
handles.bbox_delta = 10;
handles.options = MStudyHandles.options;

%store defualt data directory 
handles.data_dir = MStudyHandles.options.path;
filepath_Callback(handles.filepath, eventdata, handles);
handles.browse_press = 0;

% UIWAIT makes PointPropagatorV1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% add a continuous value change listener
if ~isfield(handles,'hListener')
    handles.hListener = ...
    addlistener(handles.time_slider,'ContinuousValueChange',@time_slider_Callback);
end

set(handles.current_image,'NextPlot','replacechildren');
%set button down function for image window
%set(handles.current_image, 'ButtonDownFcn', {@pick_points,handles});

handles.lastSliderVal = get(handles.time_slider,'Value');
% Choose default command line output for PointPropagatorV1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = PointPropagatorV1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure

function filepath_Callback(hObject, eventdata, handles)
% hObject    handle to filepath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(hObject, 'String', handles.data_dir);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function filepath_CreateFcn(hObject, eventdata, handles)
% hObject    handle to filepath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in browse_button.
function browse_button_Callback(hObject, eventdata, handles)
% hObject    handle to browse_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function current_cam_Callback(hObject, eventdata, handles)
% hObject    handle to current_cam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
current_cam = str2double(get(hObject,'String'));
start_frame = handles.Cam(current_cam).start_frame;
end_frame = handles.Cam(current_cam).end_frame;
fs_c = handles.options.fs_c;

if ~isfield(handles.Cam(current_cam),'sync_del')
    set(handles.sync_del,'String','NA')
elseif isempty(handles.Cam(current_cam).sync_del)
    set(handles.sync_del,'String','NA')
else
    set(handles.sync_del,'String',num2str(handles.Cam(current_cam).sync_del*fs_c))
end

% If the frames have not been imported, then import them.
if ~isfield(handles.Cam(current_cam),'frame')
    handles.Cam(current_cam).frame = [];
end

if isempty(handles.Cam(current_cam).frame)
    cam_str = num2str(current_cam);
    while length(cam_str)<3
        cam_str = ['0',cam_str];
    end
    
    handles.Cam(current_cam).frame = double(rgb2gray(imread([handles.data_dir,filesep,'Cam',cam_str,filesep,num2str(start_frame),'.png'])))/255;
   
    handles.Cam(current_cam).nframes = end_frame - start_frame + 1;
    handles.Cam(current_cam).rot_img = 0;
    if isempty(handles.Cam(current_cam).pts)
        handles.Cam(current_cam).pts = NaN*ones(2,handles.Cam(current_cam).nframes);
    end
end

%if the camera selected has a start_frame greater than the current time
%value, reset the start_frame value and current timestep to admissible values
if handles.Cam(current_cam).start_frame > get(handles.time_slider,'Value')
    set(handles.time_slider,'Value',handles.Cam(current_cam).start_frame+1);
    set(handles.current_timestep,'String',num2str(handles.Cam(current_cam).start_frame+1));
    set(handles.start_frame,'String',num2str(handles.Cam(current_cam).start_frame));
    set(handles.time_slider, 'Min', handles.Cam(current_cam).start_frame);
else %otherwise, just set to new value
    set(handles.time_slider, 'Min', handles.Cam(current_cam).start_frame);
    set(handles.start_frame, 'String', num2str(handles.Cam(current_cam).start_frame));
end

%if the camera selected has an end_frame greater than the current time
%value, reset the end_frame value and current timestep to admissible values
if handles.Cam(current_cam).end_frame < get(handles.time_slider,'Value')
    set(handles.time_slider,'Value',handles.Cam(current_cam).end_frame-1);
    set(handles.current_timestep,'String',num2str(handles.Cam(current_cam).end_frame-1));
    set(handles.end_frame,'String',num2str(handles.Cam(current_cam).end_frame));
    set(handles.time_slider, 'Max', handles.Cam(current_cam).end_frame);
else %otherwise, just set to new value
    set(handles.time_slider, 'Max', handles.Cam(current_cam).end_frame);
    set(handles.end_frame, 'String', num2str(handles.Cam(current_cam).end_frame));
end

set(handles.project_epipolar,'Value',handles.Cam(current_cam).rot_img);
guidata(hObject, handles);
clear vidObj
current_timestep_Callback(handles.current_timestep,eventdata,handles);

% Hints: get(hObject,'String') returns contents of current_cam as text
%        str2double(get(hObject,'String')) returns contents of current_cam as a double

% --- Executes during object creation, after setting all properties.
function current_cam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to current_cam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function current_timestep_Callback(hObject, eventdata, handles)
% hObject    handle to current_timestep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%set(handles.project_epipolar,'Value', 0)
tstep = str2double(get(hObject,'String'));
current_cam = str2double(get(handles.current_cam,'String'));
cam_str = ['Cam',num2str(current_cam)];
options = handles.options;

while cam_str<6
    cam_str = ['0',cam_str];
end

handles.Cam(current_cam).frame = imread([options.path,filesep,cam_str,filesep,num2str(tstep),'.png']);

if tstep < get(handles.time_slider,'Min')
    tstep = get(handles.time_slider,'Min');
    set(hObject,'String', num2str(tstep));
elseif tstep > get(handles.time_slider,'Max')
    tstep = get(handles.time_slider,'Max');
    set(hObject,'String', num2str(tstep));
end

set(handles.time_slider,'Value',tstep);
%current_cam = str2double(get(handles.current_cam,'String'));
%h = imshow(handles.Cam(current_cam).frames(:,:,:,tstep-handles.Cam(current_cam).start_frame+1),'Parent',handles.current_image);
%handles.image_handle = h;
handles = show_image(hObject, handles);
if get(handles.project_epipolar,'Value')
    project_epipolar_Callback(handles.project_epipolar,eventdata,handles)
end
set(handles.image_handle, 'ButtonDownFcn', {@current_image_ButtonDownFcn, handles});
set(handles.image_handle, 'Interruptible', 'on');
%set(handles.image_handle, 'HitTest', 'off');
% set(h, 'ButtonDownFcn', {@pick_points,handles});
% set(handles.current_image,'Children',h);
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of current_timestep as text
%        str2double(get(hObject,'String')) returns contents of current_timestep as a double

% --- Executes during object creation, after setting all properties.
function current_timestep_CreateFcn(hObject, eventdata, handles)
% hObject    handle to current_timestep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function current_point_label_Callback(hObject, eventdata, handles)
% hObject    handle to current_point_label (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of current_point_label as text
%        str2double(get(hObject,'String')) returns contents of current_point_label as a double

if get(handles.project_epipolar,'Value')
    kids = get(handles.current_image, 'Children');
    delete(kids)
    current_timestep_Callback(handles.current_timestep, eventdata, handles)
    project_epipolar_Callback(handles.project_epipolar,eventdata,handles);
end

% --- Executes during object creation, after setting all properties.
function current_point_label_CreateFcn(hObject, eventdata, handles)
% hObject    handle to current_point_label (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in save_button.
function save_button_Callback(hObject, eventdata, handles)
% hObject    handle to save_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% for cc = 1:length(handles.Cam)
%     handles.Cam(cc).frames = [];
% end
MStudyHandles.Cam = handles.Cam;
setappdata(0,'MStudyHands', MStudyHandles);

for cc = 1:length(handles.Cam)
    handles.Cam(cc).frame = [];
end
fprintf('Saving data ... do not close MATLAB or MOTIONSTUDY ... \n')
%save([handles.options.path,filesep,'CamStruct.mat'], '-struct', 'handles','Cam', '-v7.3')
cams = which_cams(handles.Cam);
cams = [cams(1), cams(end)];
Cam = handles.Cam(cams(1):cams(2));
save([handles.options.path,filesep,'CamStruct.mat'], 'Cam', 'cams')
fprintf('The session has been saved.\n')
%varargout{1} = handles.Cam;
%save([handles.data_dir,filesep,'CamStruct.mat'], '-struct', 'handles','Cam','-v7.3')

% --- Executes on slider movement.
function time_slider_Callback(hObject, eventdata, handles)
% hObject    handle to time_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 handles = guidata(hObject);
 % get the slider value and convert it to the nearest integer that is less
 % than this value
 newVal = round(get(hObject,'Value'));
 % set the slider value to this integer which will be in the set {1,2,3,...,12,13}
 set(hObject,'Value',newVal);
 % now only do something in response to the slider movement if the 
 % new value is different from the last slider value
 if newVal ~= handles.lastSliderVal
     % it is different, so we have moved up or down from the previous integer
     % save the new value
     handles.lastSliderVal = newVal;
     guidata(hObject,handles);
    % display the current value of the slider
    %disp(['at slider value ' num2str(get(hObject,'Value'))]);
    set(handles.current_timestep,'String',num2str(newVal))
    current_cam = str2double(get(handles.current_cam,'String'));
    cam_str = ['Cam',num2str(current_cam)];
    while cam_str<6
    cam_str = ['0',cam_str];
    end
    handles.Cam(current_cam).frame = imread([options.path,filesep,cam_str,filesep,num2str(tstep),'.png']);
    handles = show_image(hObject,handles);
    %set(handles.image_handle, 'ButtonDownFcn', {@pick_points,handles});
    if ~isempty(handles.Cam(current_cam).pts);
        plot_points(handles.image_handle,handles)
    else
        set(handles.image_handle, 'ButtonDownFcn', {@current_image_ButtonDownFcn, handles});
        set(handles.image_handle, 'Interruptible', 'on');
    end
    %set(handles.image_handle, 'HitTest', 'off');
 end
 guidata(hObject, handles);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function time_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to time_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function start_frame_Callback(hObject, eventdata, handles)
% hObject    handle to start_frame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of start_frame as text
%        str2double(get(hObject,'String')) returns contents of start_frame as a double
% handles.Cam(str2double(get(handles.current_cam,'String'))).start_frame = str2double(get(hObject,'String'));
% if str2double(get(hObject,'String')) > get(handles.time_slider,'Value')
%     set(handles.time_slider,'Value', str2double(get(hObject,'String')))
%     set(handles.current_timestep,'String',get(hObject,'String'))
% end
% set(handles.time_slider,'Min',str2double(get(hObject,'String')));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function start_frame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to start_frame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function end_frame_Callback(hObject, eventdata, handles)
% hObject    handle to end_frame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% if str2double(get(hObject,'String')) < get(handles.time_slider,'Value')
%     set(handles.time_slider,'Value', str2double(get(hObject,'String')))
%     set(handles.current_timestep,'String',get(hObject,'String'))
% end
% 
% if str2double(get(hObject,'String')) <= handles.Cam(str2double(get(handles.current_cam,'String'))).nframes
%     handles.Cam(str2double(get(handles.current_cam,'String'))).end_frame = str2double(get(hObject,'String'));
%     set(handles.time_slider,'Max',str2double(get(hObject,'String')));
% else
%     handles.Cam(str2double(get(handles.current_cam,'String'))).end_frame = handles.Cam(str2double(get(handles.current_cam,'String'))).nframes;
%     set(handles.time_slider,'Max',handles.Cam(str2double(get(handles.current_cam,'String'))).nframes);
% end
guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of end_frame as text
%        str2double(get(hObject,'String')) returns contents of end_frame as a double

% --- Executes during object creation, after setting all properties.
function end_frame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to end_frame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in project_epipolar.
function project_epipolar_Callback(hObject, eventdata, handles)
% hObject    handle to project_epipolar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of project_epipolar
bbox_delta = handles.bbox_delta;
Cam = handles.Cam;                  %Get the Cam Struct 
cams = which_cams(Cam);             %Get the cam numbers with data 
current_cam = str2double(get(handles.current_cam,'String'));    %determine which camera is the current camera
pt = str2double(get(handles.current_point_label,'String'));     %determinw which point label is selected
timestep = str2double(get(handles.current_timestep,'String'));  %determine the current frame in this camera
proj_epi = get(hObject,'Value');               %determine if the project epi option is selected
set(handles.current_image,'NextPlot', 'Add');  %add the lines to the current plot
no_sync = [];
fs_c = handles.options.fs_c;
if proj_epi
    for cc = cams
        if cc == current_cam 
            continue;
        end
        
        if isempty(Cam(cc).sync_del)
            no_sync = [no_sync, cc];
            continue;
        end
        
        %determine the world time
        t_world = timestep - floor(Cam(current_cam).sync_del*fs_c);
        if Cam(current_cam).sync_del
            dt_wc = 1 - (Cam(current_cam).sync_del*fs_c-floor(Cam(current_cam).sync_del*fs_c));
        else
            dt_wc = 0;
        end
        %determine the time in the camera from which an epipolar line
        %should be projected.
        tt = t_world + floor(fs_c*Cam(cc).sync_del)- Cam(cc).start_frame + 1;
        
        if tt<1 || tt>length(Cam(cc).pts) || pt>size(Cam(cc).pts,3)
            continue;
        end        
        %tt = tt-1:tt+1; %get one timestep in either direction
        linesty = {'c','m','g'};
        for t = tt  %For each of the desired timesteps
            if Cam(cc).sync_del
                dt_wi = 1 - (Cam(cc).sync_del*fs_c-floor(Cam(cc).sync_del*fs_c));
            else
                dt_wi = 0;
            end
            dt_ci = dt_wc - dt_wi;
            d = sign(dt_ci);
            %if the points are availabe in this camera
            if ~isnan(Cam(cc).pts(:,t,pt)) & ~isnan(Cam(cc).pts(:,t+d,pt))
                if dt_ci < 0 
                    pt1 = Cam(cc).pts(:,t+d,pt);
                    pt2 = Cam(cc).pts(:,t,pt);
                else
                    pt1 = Cam(cc).pts(:,t,pt);
                    pt2 = Cam(cc).pts(:,t+d,pt);
                end

                %remove distortion from the points
                p1_r = rm_distortion(pt1,Cam(cc).K_dist,Cam(cc).fc_dist,Cam(cc).cc_dist,Cam(cc).alpha_c_dist,Cam(cc).kc_dist);%rm_distortion(x_p, K, fc, prin_p, skew, dist_c)
                p2_r = rm_distortion(pt2,Cam(cc).K_dist,Cam(cc).fc_dist,Cam(cc).cc_dist,Cam(cc).alpha_c_dist,Cam(cc).kc_dist);
                %interpolate the point in undist pix coords
                if dt_ci < 0 
                    p_r = p1_r + (p2_r - p1_r)*(1+dt_ci);
                else
                    p_r = p1_r + (p2_r - p1_r)*dt_ci;
                end
                
                %determine the epipolar line parameters
                epi_line = epipolar_line(Cam(cc).H,Cam(current_cam).H,Cam(cc).K\[p_r;1]);
                %project bounding box into frame
                b_box = Cam(current_cam).b_box(timestep - Cam(current_cam).start_frame+1,:); %determine the bounding box
                pts_bbox = [b_box(1:2)', b_box(1:2)' + b_box(3:4)';ones(1,2)];
                
                pts_bbox_ud(:,1) = Cam(current_cam).K\[rm_distortion(pts_bbox(1:2,1),Cam(current_cam).K_dist,Cam(current_cam).fc_dist,Cam(current_cam).cc_dist,Cam(current_cam).alpha_c_dist,Cam(current_cam).kc_dist);1];%rm_distortion(x_p, K, fc, prin_p, skew, dist_c)
                pts_bbox_ud(:,2) = Cam(current_cam).K\[rm_distortion(pts_bbox(1:2,2),Cam(current_cam).K_dist,Cam(current_cam).fc_dist,Cam(current_cam).cc_dist,Cam(current_cam).alpha_c_dist,Cam(current_cam).kc_dist);1];
                x = linspace(pts_bbox_ud(1,1),pts_bbox_ud(1,2),300);  %define x retinal coords for line
                y = epi_line(2)/epi_line(1)*(x-epi_line(3))+epi_line(4);       %compute y retinal coords for line
                x_n = [x;y];                         %combine x and y into points
                kc = Cam(current_cam).kc_dist;               %distort points
                xd = [zeros(2,size(x_n,2));ones(1,size(x_n,2))]; %create place holder for distorted points
                for pp = 1:size(x_n,2)  %compute the distortions
                    pt_n = x_n(:,pp);
                    r = norm(pt_n);
                    xd(1:2,pp) = (1+kc(1)*r^2+kc(2)*r^4+kc(5)*r^6)*pt_n+[2*kc(3)*pt_n(1)*pt_n(2)+ kc(4)*(r^2+2*pt_n(1)^2);2*kc(4)*pt_n(1)*pt_n(2)+kc(3)*(r^2+2*pt_n(2)^2)];
                end
                pts = Cam(current_cam).K*xd;  %compute distorted pixel coords
                
                indx = isnan(pts) | repmat(pts(1,:)<b_box(1), 3, 1) | repmat(pts(1,:)>(b_box(1)+b_box(3)), 3, 1) ...
                            | repmat(pts(2,:)<b_box(2), 3, 1) | repmat(pts(2,:)>(b_box(2)+b_box(4)), 3, 1);
                indx = sum(indx)==0;     %Trim the line to fit in the bounding box
                pts = pts(:,find(indx)); 
                if ~isempty(pts)      %If there are points left in the matrix, plot them.
                    h = plot(handles.current_image, pts(1,:)'-b_box(1)+bbox_delta,pts(2,:)'-b_box(2)+bbox_delta,'-c');%linesty{t-tt(2)+2});
                    set(h, 'HitTest', 'off');
                    h = text(pts(1,1)'-b_box(1),pts(2,1)'-b_box(2),num2str(cc), 'color','c');% linesty{t-tt(2)+2});
                    set(h, 'HitTest', 'off');
                end
            end
        end
    end
    if ~isempty(no_sync)
        fprintf('No sync data available for the following cameras:\n')
        fprintf('%d\t' , no_sync)
        fprintf('\n Skipping these cameras. \n')
    end
else
    kids = get(handles.current_image, 'Children');
    delete(kids)
    current_timestep_Callback(handles.current_timestep, eventdata, handles);    
end

%--------------------------------------------------------------------------
%                         My Processing Functions
%--------------------------------------------------------------------------
% --- Executes on mouse press over axes background.
function current_image_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to project_epipolar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% PICK_POINTS specifies new points 
point = get(get(hObject, 'Parent'),'Currentpoint');

%determine camera number, time step, and point number being selected
cam = str2double(get(handles.current_cam,'String'));
timestep = str2double(get(handles.current_timestep,'String'))-handles.Cam(cam).start_frame+1;
point_num = str2double(get(handles.current_point_label,'String'));
%determine number of total steps and number of total points
[~, nsteps, npts] = size(handles.Cam(cam).pts);
%if the zoom feature is being used, compute the appropriate offset
% if handles.Cam(cam).zoom
%         x = handles.Cam(cam).b_box(timestep,1)-1-handles.bbox_delta;
%         y = handles.Cam(cam).b_box(timestep,2)-1-handles.bbox_delta;
% else %otherwise, the offset is nothing
    x = 0;
    y = 0;
% end

%if a new point number has been specified, then add page to the point
%matrix
if npts<point_num
   handles.Cam(cam).pts(:,:,(npts+1):point_num) = NaN*ones(2,nsteps,point_num-npts);
end

persistent chk %delcare a persistent variable to determine if a double click has been used
if isempty(chk) %if no immediately previous click
    chk = 1;    %then chk variable is high
    pause(1); %Add a delay to distinguish single click from a double click
    if chk == 1 %if there was only one click
        %compute the appropriate location based on the new click
        locs(:,1) = handles.Cam(cam).features{timestep}.Location(:,1)-x;
        locs(:,2) = handles.Cam(cam).features{timestep}.Location(:,2)-y;
        %compute the distance to the click location 
        dist = locs - repmat(point(1,1:2),size(locs,1),1);
        delta = sum(dist.*dist,2);
        [val,I] = min(delta);
        if val<10^2     %if there is a point within the threshold, use that point
            handles.Cam(cam).pts(:,timestep,point_num) = locs(I,:)'+[x;y];
        end
        plot_points(hObject, handles);
    else
        
    end
    chk = []; %empty the click check
     %plot the new point and label
else %if the register was full, the this was a double click
    chk = 2; %there were two clicks
    handles.Cam(cam).pts(:,timestep,point_num) = point(1,1:2)'+[x;y]; %take the click location 
    plot_points(hObject, handles); %plot points
end

function plot_points(hObject, handles)
cam = str2double(get(handles.current_cam,'String'));
npts = size(handles.Cam(cam).pts,3);
timestep = str2double(get(handles.current_timestep,'String')) - handles.Cam(cam).start_frame+1;

if any(any(any(~isnan(handles.Cam(cam).pts))))
    set(handles.current_image,'NextPlot', 'Add');
    ax_child = get(handles.current_image,'Children');
    child_types = get(ax_child,'Type');

    if iscell(child_types)
        for ii = 1:length(child_types)
            if ~isempty(regexp('line',child_types{ii})) || ~isempty(regexp('text',child_types{ii}));
                delete(ax_child(ii));
            end
        end
    end

%     if handles.Cam(cam).zoom
%         x = handles.Cam(cam).b_box(timestep,1)-1-handles.bbox_delta;
%         y = handles.Cam(cam).b_box(timestep,2)-1-handles.bbox_delta;
%         plot(handles.current_image, reshape(handles.Cam(cam).pts(1,timestep,:)-x,1,[]), reshape(handles.Cam(cam).pts(2,timestep,:)-y,1,[]),'r+');
%         for pp = 1:npts
%             if ~isnan(handles.Cam(cam).pts(:,timestep,pp))
%                 h = text(handles.Cam(cam).pts(1,timestep,pp)-x,handles.Cam(cam).pts(2,timestep,pp)-y,num2str(pp), 'Color', 'r', 'Parent', handles.current_image, 'HorizontalAlignment', 'Right');
%                 set(h, 'HitTest', 'off');
%             end
%         end
%     else
        plot(handles.current_image, reshape(handles.Cam(cam).pts(1,timestep,:),1,[]), reshape(handles.Cam(cam).pts(2,timestep,:),1,[]),'ro');
        for pp = 1:npts
            if ~isnan(handles.Cam(cam).pts(:,timestep,pp))
                h = text(handles.Cam(cam).pts(1,timestep,pp),handles.Cam(cam).pts(2,timestep,pp),num2str(pp), 'Color', 'r', 'Parent', handles.current_image, 'HorizontalAlignment', 'Right');
                set(h, 'HitTest', 'off');
            end
        end
%     end

    set(handles.current_image,'NextPlot', 'replacechildren');
    set(handles.image_handle, 'ButtonDownFcn', {@current_image_ButtonDownFcn, handles});
    set(handles.image_handle, 'Interruptible', 'on');
    guidata(handles.figure1,handles);
    
end

function handles = show_image(hObject, handles)
%all of this functionality allows for the user to click points on the
%image.  However, these computations require significant time and cause
%scrolling to lag.  Pursue more efficient format for point picking in the
%future.
cam = str2double(get(handles.current_cam,'String'));
tstep = round(get(handles.time_slider,'Value'))-handles.Cam(cam).start_frame+1;

% if isfield(handles.Cam(cam),'b_box')
%     b_box = handles.Cam(cam).b_box(tstep,:)+[-handles.bbox_delta, -handles.bbox_delta, 2*handles.bbox_delta, 2*handles.bbox_delta];
%     if b_box(1)<1
%         b_box(1) = 1;
%     end
%     if b_box(2)<1
%         b_box(2) = 1;
%     end
%     if b_box(1)+b_box(3)>1280
%         b_box(3) = 1280-b_box(1);
%     end
%     if b_box(2)+b_box(4)>720
%         b_box(4) = 720-b_box(2);
%     end
%     h = imshow(handles.Cam(cam).frame,'Parent',handles.current_image);
%     axis([b_box(1),b_box(1)+b_box(3),b_box(2),b_box(2)+b_box(4)])
%     handles.Cam(cam).zoom = 1;
% else
    h = imshow(handles.Cam(cam).frame,'Parent',handles.current_image);
% end
handles.image_handle = h;

if ~isempty(handles.Cam(cam).pts) && any(any(~isnan(handles.Cam(cam).pts(:,tstep,:))))
    plot_points(handles.image_handle,handles)
else
    set(handles.image_handle, 'ButtonDownFcn', {@current_image_ButtonDownFcn, handles});
    set(handles.image_handle, 'Interruptible', 'on');
end

set(gcf,'ToolBar','figure')

% --- Executes on button press in track_pts.
function track_pts_Callback(hObject, eventdata, handles)
% hObject    handle to track_pts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Cam = track_points_im(handles.Cam,handles.options.path);
guidata(hObject,handles);


% --- Executes on button press in delete_pt.
function delete_pt_Callback(hObject, eventdata, handles)
% hObject    handle to delete_pt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cam = str2double(get(handles.current_cam,'String'));
timestep = get(handles.time_slider,'value') - handles.Cam(cam).start_frame+1;
pt_num = str2double(get(handles.current_point_label,'String'));

handles.Cam(cam).pts(:,timestep,pt_num) = [NaN;NaN];
plot_points(hObject, handles)
guidata(hObject,handles);


% --- Executes on button press in delete_multi.
function delete_multi_Callback(hObject, eventdata, handles)
% hObject    handle to delete_multi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cam = str2double(get(handles.current_cam,'String'));
pt_nums = input('Which pts would you like to delete?:');
timesteps = input('Which timesteps would you like to delete?:')- handles.Cam(cam).start_frame+1;
for pp = pt_nums
    handles.Cam(cam).pts(:,timesteps,pp) = NaN*ones(2,length(timesteps));
end
plot_points(hObject, handles)
guidata(hObject,handles);



function sync_del_Callback(hObject, eventdata, handles)
% hObject    handle to sync_del (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sync_del as text
%        str2double(get(hObject,'String')) returns contents of sync_del as a double


% --- Executes during object creation, after setting all properties.
function sync_del_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sync_del (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in predict_epipolar.
function predict_epipolar_Callback(hObject, eventdata, handles)
% hObject    handle to predict_epipolar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

bbox_delta = handles.bbox_delta;
Cam = handles.Cam;                  %Get the Cam Struct 
cams = input('In which cams would you like to search for points?:');             %Get the cam numbers with data 
current_cam = str2double(get(handles.current_cam,'String'));    %determine which camera is the current camera
%pt = str2double(get(handles.current_point_label,'String'));     %determinw which point label is selected
timestep = str2double(get(handles.current_timestep,'String'));  %determine the current frame in this camera
proj_epi = get(hObject,'Value');               %determine if the project epi option is selected
set(handles.current_image,'NextPlot', 'Add');  %add the lines to the current plot
no_sync = [];
fs_c = handles.options.fs_c;

%find available points for prediction in this view (pts seen in at 
%least two other cameras)
pts_avail_mat = [];
ncams = 0;
cams_left = [];
max_pt = 0;
plot_obj = [];
txt_obj = [];
for cc = cams
    p_r{cc == cams} = [];
    
    if cc == current_cam 
        continue;
    end

    if isempty(Cam(cc).sync_del)
        no_sync = [no_sync, cc];
        continue;
    end

    %determine the world time
    t_world = timestep - floor(Cam(current_cam).sync_del*fs_c);
    if Cam(current_cam).sync_del
        dt_wc = 1 - (Cam(current_cam).sync_del*fs_c-floor(Cam(current_cam).sync_del*fs_c));
    else
        dt_wc = 0;
    end
    %determine the time in the camera from which an epipolar line
    %should be projected.
    tt = t_world + floor(fs_c*Cam(cc).sync_del)- Cam(cc).start_frame + 1;

    if tt<1 || tt>length(Cam(cc).pts) %|| pt>size(Cam(cc).pts,3)
        continue;
    end
    
    if Cam(cc).sync_del
        dt_wi = 1 - (Cam(cc).sync_del*fs_c-floor(Cam(cc).sync_del*fs_c));
    else
        dt_wi = 0;
    end
    dt_ci = dt_wc - dt_wi;
    d = sign(dt_ci);
    %if the points are availabe in this camera
    pts_avail = [1 0]*squeeze((~isnan(Cam(cc).pts(:,tt,:)) & ~isnan(Cam(cc).pts(:,tt+d,:))));
    if ~any(pts_avail)
        continue
    end
    ncams = ncams + 1;
    cams_left = [cams_left, cc];
    if size(pts_avail,2)>size(pts_avail_mat,2)
        pts_avail_mat = [pts_avail_mat, zeros(size(pts_avail_mat,1),length(pts_avail)-size(pts_avail_mat,2))];
    end
    
    p_r{cams == cc} = NaN*zeros(2,length(pts_avail));
    for pt = find(pts_avail)
        if dt_ci < 0 
            pt1 = Cam(cc).pts(:,tt+d,pt);
            pt2 = Cam(cc).pts(:,tt,pt);
        else
            pt1 = Cam(cc).pts(:,tt,pt);
            pt2 = Cam(cc).pts(:,tt+d,pt);
        end

        %remove distortion from the points
        p1_r = rm_distortion(pt1,Cam(cc).K_dist,Cam(cc).fc_dist,Cam(cc).cc_dist,Cam(cc).alpha_c_dist,Cam(cc).kc_dist);%rm_distortion(x_p, K, fc, prin_p, skew, dist_c)
        p2_r = rm_distortion(pt2,Cam(cc).K_dist,Cam(cc).fc_dist,Cam(cc).cc_dist,Cam(cc).alpha_c_dist,Cam(cc).kc_dist);
        %interpolate the point in undist pix coords
        if dt_ci < 0 
            p_r{cams == cc}(:,pt) = p1_r + (p2_r - p1_r)*(1+dt_ci);
        else
            p_r{cams == cc}(:,pt) = p1_r + (p2_r - p1_r)*dt_ci;
        end
        
    end
    
    if size(pts_avail_mat,2)<length(pts_avail)
        pts_avail_mat = [pts_avail_mat, zeros(size(pts_avail_mat,1),length(pts_avail))];
    end
    
    pts_avail_mat(ncams,1:length(pts_avail)) = pts_avail;
    
    if max_pt<find(pts_avail,1,'last')
        max_pt = find(pts_avail,1,'last');
    end
end

pts2est  = find(sum(pts_avail_mat)>=2);
pts_have = find(~isnan(sum(squeeze(Cam(current_cam).pts(:,timestep - Cam(current_cam).start_frame+1,:)),1)));
pts2est = setdiff(pts2est,pts_have);

for pt = pts2est
    lines = [];
    for cc = cams
        if pt>size(p_r{cams == cc},2)
            continue;
        end
        phi = p_r{cc == cams}(:,pt);
        if any(isnan(phi))
            continue;
        end
        l = epipolar_line(Cam(cc).H, Cam(current_cam).H,Cam(cc).K\[phi;1]);
        lines = [lines;l];
    end
    b_box = Cam(current_cam).b_box(timestep - Cam(current_cam).start_frame +1,:);
    if size(lines,1)>=2
       kc = Cam(current_cam).kc_dist;

       p = [eye(2),zeros(2,1)]*(Cam(cc).K_dist\Cam(cc).K*[nearest2lines(lines);1]);
       r = norm(p);
       
       xd = (1+kc(1)*r^2+kc(2)*r^4+kc(5)*r^6)*p+[2*kc(3)*p(1)*p(2)+ kc(4)*(r^2+2*p(1)^2);2*kc(4)*p(1)*p(2)+kc(3)*(r^2+2*p(2)^2)];
       point =[eye(2),zeros(2,1)]*Cam(current_cam).K_dist*[xd;1];
       ncands = size(Cam(current_cam).features{timestep - Cam(current_cam).start_frame+1}.Location,1);
       delta = Cam(current_cam).features{timestep - Cam(current_cam).start_frame+1}.Location - repmat(point',ncands,1);
       dist = sum(delta.^2,2);
       [val,pt_num] = min(dist);
       if val<10^2
           feat_cands(:,pt) = Cam(current_cam).features{timestep - Cam(current_cam).start_frame+1}.Location(pt_num,:)';
       else
           feat_cands(:,pt) = [NaN;NaN];
       end
    else
       feat_cands(:,pt) = [NaN;NaN];
    end
    h2 = text(double(feat_cands(1,pt)'),double(feat_cands(2,pt)'),num2str(pt),'color', 'y');
    txt_obj = [txt_obj, h2];
    set(h2, 'HitTest', 'off');
end

%plot all identified features to debug
%plot(handles.current_image,Cam(current_cam).features{timestep - Cam(current_cam).start_frame+1}.Location(:,1)'-b_box(1)+bbox_delta,Cam(current_cam).features{timestep - Cam(current_cam).start_frame+1}.Location(:,2)'-b_box(2)+bbox_delta, 'om')
h1 = plot(handles.current_image,feat_cands(1,pts2est)',feat_cands(2,pts2est)','*y') ;
plot_obj = [plot_obj, h1];
set(h1, 'HitTest', 'off');

%Prompt the user which points should be kept and store them in the 
pts_usr = input('Which points would you like to keep?:');
for pp = pts_usr
    Cam(current_cam).pts(:,timestep - Cam(current_cam).start_frame+1,pp) = feat_cands(:,pp);
end    

%delete the predicted points, and plot all points again.
if ~isempty(pts_usr)
    delete(plot_obj)
end
delete(txt_obj)
handles.Cam = Cam;
plot_points(hObject, handles);
guidata(hObject, handles)


%find closest point
