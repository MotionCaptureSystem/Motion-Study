function txt=myCallback(~,event_obj,pts_info_obj,CSP)
% Customizes text of data tips


handles=getappdata(0,'handles');
pos = get(event_obj,'Position');
cam_string = handles.options.Obsr{round(pos(2)),round(pos(1))};
Clicked_time=num2str(str2double(handles.options.bar_starttime)+round(pos(1))-1);

persistent bchk %delcare a persistent variable to determine if a double click has been used
if isempty(bchk) %if no immediately previous click
    bchk =1;    %then chk variable is high
    pause(1); %Add a delay to distinguish single click from a double click
    if bchk == 1 %if there was only one click
        
        s = ['Captured by Cam(s): ' cam_string];

        set(handles.figure1.CurrentAxes.Title,'String',s);

        txt = {['Frame : ',Clicked_time],...
              ['Point : ',num2str(round(pos(2)))],...
              ['Obers # : ',num2str(CSP(round(pos(2)),round(pos(1))))]
              };
          
        disp({'Frame : ',Clicked_time,...
              'Point : ',num2str(round(pos(2))),...
              'Obers # : ',num2str(CSP(round(pos(2)),round(pos(1))))});
    end
    bchk=[];
else %if the register was full, the this was a double click
    bchk=2;
    clc;
    %set the time and camera number to be used in PointPropagator.
    handles.options.current_timestep=Clicked_time;
    
    setappdata(0,'MStudyHands',handles);
    
%      txt = {['Frame : ',Clicked_time],...
%               ['Point : ',num2str(round(pos(2)))],...
%               ['Obers # : ',num2str(CSP(round(pos(2)),round(pos(1))))]
%               };
% %     
      disp({'Frame : ',Clicked_time,...
              'Point : ',num2str(round(pos(2))),...
              'Obers # : ',num2str(CSP(round(pos(2)),round(pos(1))))});
          
          
    PointPropagatorV1

%     set(pts_info_obj,'Enable','off')
    handles.options.current_timestep=[];
    bchk=[];
    
   
          
end

end

