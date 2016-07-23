function BarGraph(handles)

addpath('Calibration', 'Common', 'FeatIdent','ImageProc','MotionEst','Results','PtsInfo')
addpath(['.',filesep,'MotionEst',filesep,'Init'],...
        ['.',filesep,'MotionEst',filesep,'Models'],...
        ['.',filesep,'MotionEst',filesep,'Stereo'],...
        ['.',filesep,'MotionEst',filesep,'TrajEst'])


cam_list=[];
%---This snippet of codes find the maximum number of points being used.---%
npts=0;

for cc= 1:length(handles.Cam)
    if size(handles.Cam(cc).pts,3)>npts
         npts=size(handles.Cam(cc).pts,3);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if npts>100
    npts=50;  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------Get start and end time from user input----------------%

starttime=str2double(handles.options.bar_starttime);
endtime=str2double(handles.options.bar_endtime);

if isnan(starttime)
    starttime = [];
end
if isnan(endtime)
    endtime = [];
end

cc = 0;
while isempty(starttime)
    cc = cc+1;
    starttime = handles.Cam(cc).start_frame;
end
cc = 0;
while isempty(endtime)
    cc = cc+1;
    endtime = handles.Cam(cc).end_frame;
end

set(handles.Start_time,'string',num2str(starttime));
set(handles.edit2,'string',num2str(endtime));

tottime=endtime-starttime+1;

%--------------------------------------------------------------------%
%cam_num= [301 302 303 310 312 318 320 325 333]; 
%--------------------------------------------------------------------%
%This snippet of codes tell us if there are points being picked in
%cameras, thus we could get the total camera numbers.
for cc= 1:length(handles.Cam)
    if ~isempty(~isnan(handles.Cam(cc).pts))
         cam_list = [cam_list, cc];
    end
end
cn2=size(cam_list,2);

P=zeros(npts,tottime,cn2);
%--------------------------------------------------------------------%

for k = 1:cn2    
    
    %--------------------------------------------------------------------%
    %Because each camera starts in different frame, which means different
    %start frame and end frame, we have to assign frame number
    %individually.
    
%     if cam_list(k)==318
%         test=1;
%     end
    
    frame_num = starttime- handles.Cam(cam_list(k)).start_frame+floor(handles.Cam(cam_list(k)).sync_del*119.8) +1 : starttime-handles.Cam(cam_list(k)).start_frame+floor(handles.Cam(cam_list(k)).sync_del*119.8) +tottime;
    
    %--------------------------------------------------------------------%
    fn2=size(frame_num,2);% loop cam 
    for j = 1:fn2  %loop through frame #
    
        pts_dim=size(handles.Cam(cam_list(k)).pts(1,j,:),3);
        for i=1 : pts_dim                                                      %loop through pts; 
            if isnan(handles.Cam(cam_list(k)).pts(1,j,i))==0;
                P(i,j,k)=1;
            end
        end   
    end
end
%End of building the interface
%%-----------------------------------------------------------------------%%



[d1,d2,d3]=size(P);         %d1 # of points;d2 # of frames; d3 # of cameras;
CSP=zeros(d1,d2);            %Camera See Points(CSP) matrix, 1st column pts#, 
                            %2nd how many cameras see it.  
                            
Obsr=cell(d1,d2);

for j=1 : d2;                 %First loop through pts;
    for i=1 : d1;             %Given a specific point, loop through frame #
    n=0; 
        for k= 1 : d3;        %Given both specific point#&frame#, loop cam       
            if P(i,j,k)==1;
                Obsr(i,j)= strcat(Obsr(i,j),' ',num2str(cam_list(k)),',');
                n=n+1;
            end
        end
    CSP(i,j)= n;       %For a specific pt% time step(represented by frame#)
                       %how many cams see it. 
    end
end                            

%Instead of using axes directly on the MotionStudy.fig, we are plotting on
%subplot of the MotionStudy.fig 


%get the current figure handles.
s=gca;

positionVector=[0, 0.1, 0.7, 0.7];
set(s,'Position',positionVector)


b=bar3(s,CSP);

hold on;
% patch(s,[0,tottime+1,tottime+1,0], [0,0,npts+1,npts+1], [2,2,2,2], 'r');

ylabel(s,'point number','FontSize',12);
xlabel(s,'frame number (time step)','FontSize',12);
zlabel(s,'number of observation','FontSize',12);
title(s,'point observation');

for n=1:numel(b)

     cdata=get(b(n),'zdata');
     cdata=repmat(max(cdata,[],2),1,4);
     set(b(n),'cdata',cdata,'facecolor','flat')

end

view(3);  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   
set(s,'XTickLabel',{num2str(floor(starttime+tottime/4*1)),num2str(floor(starttime+tottime/4*2)),num2str(floor(starttime+tottime/4*3)),num2str(endtime)});
set(s,'XTickMode','manual');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

handles.options.Obsr=Obsr;
fig_bar=handles.figure1;

% fig_bar.CurrentAxes.XLim=fig_bar.CurrentAxes.XLim+starttime;

setappdata(0,'handles',handles);

pts_info_obj = datacursormode(fig_bar);

set(pts_info_obj,'UpdateFcn',{@myCallback,pts_info_obj,CSP});
%------------------------------------------------------------------------%
% set(fig, 'ButtonDownFcn', @myCallback);
%------------------------------------------------------------------------%

end
