function txt = info_handle(~,event_obj)

set(event_obj.Target,'ButtonDownFcn',@myCallback);
txt=myCallback(event_obj);
test=5;
% Customizes text of data tips
% persistent bchk %delcare a persistent variable to determine if a double click has been used
% 
% handles=getappdata(0,'handles');
% pos = get(event_obj,'Position');
% cam_string = handles.options.Obsr{round(pos(2)),round(pos(1))};
% 
% if isempty(bchk) %if no immediately previous click
%     bchk = 1;    %then chk variable is high
%     pause(1); %Add a delay to distinguish single click from a double click
%     if bchk == 1 %if there was only one click
%         
%         s = ['Captured by Cam(s): ' cam_string];
% 
%         set(handles.figure1.CurrentAxes.Title,'String',s);
% 
%         txt = {['Frame : ',num2str(round(pos(1)))],...
%               ['Point : ',num2str(round(pos(2)))],...
%               ['Obers # : ',num2str(round(pos(3)))]
%               };
%     end
% else %if the register was full, the this was a double click
%     if bchk == 2; %there were two clicks
%     handles.options.current_cursor=[round(pos(1)),round(pos(2))];
%     end
% end
% 
% bchk = []; %empty the click check
