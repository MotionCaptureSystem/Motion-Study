function cam_list = which_cams(Cams)
%%Determines which cameras have been imported. CAMS is a structure of
%%camera data. The required subfield is CAM.STARTFRAME.  This field is
%%automatically populated when a camera is imported.  CAM_LIST is a row
%%vector of the cams which have been successfully imported.

ncams = length(Cams);
cam_list = [];
%Run loop for length of Cam structure
for cc = 1:ncams
    %If the camera has been imported, store the camera number
    if ~isempty(Cams(cc).start_frame)
        cam_list = [cam_list, cc];
    end
end