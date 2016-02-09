function camstruct = load_caltech_intrinsic(camstruct, options, cams)
%LOAD_CALTECH_INTRINSIC     -loads the intrinsic parameters from the
%CalTech calibration toolbox.  Parameters must be stored in a MAT file
%which contains KK (intrinsic camera parameters) and KC (vector of
%distortion coefficients.

%set the location to get the parameters from 
path = [options.path,filesep,'..',filesep,'Calibration_run',filesep,'Intrinsic',filesep,'CalTech'];
%for the desired cameras, import the parameters
for c = cams
    load([path,filesep,'Cam',num2str(c),filesep,'int_cam',num2str(c),'.mat'],'KK','kc','alpha_c','fc','cc')
    camstruct(c).K = KK;
    camstruct(c).kc = kc;
    camstruct(c).fc = fc;
    camstruct(c).cc = cc;
    camstruct(c).alpha_c = alpha_c;
end

    