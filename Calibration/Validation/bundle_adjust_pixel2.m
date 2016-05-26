function camstruct_adjusted = bundle_adjust_pixel2(camstruct, options)
%BUNDLE_ADJUST      -This function can be run after an import utility
%which creates the Ring(1).Cam structure.  This code adjusts the external
%camera calibration by minimizing the error between triangulation performed
%with any pair of cameras.  

%If bundle adjustment is not desired, exit this function
if ~options.ba
    camstruct_adjusted = camstruct;
    return
end
cams = options.cams;
ncam = length(cams);

%Compile Original Extrinsics and Plot
H = zeros(4,4,ncam);
for cc = 1:ncam
    H(:,:,cc) = camstruct(cams(cc)).H;
end
figure
hold on
CFPlot(H, 0.1)
axis equal

savefile = [options.path,filesep,'bundle_adjust.mat'];
%Determine if Bundle Adjust Already extists
if (options.ba == 1 || options.ba ==3)
    load(savefile)
    camstruct_adjusted = camstruct;
    for cc = cams
        camstruct_adjusted(cc).H = camstruct_exp(cc).H;
        camstruct_adjusted(cc).T = camstruct_exp(cc).T;
        camstruct_adjusted(cc).om = camstruct_exp(cc).om;
    end
    clear camstruct_exp
    for cc = 1:ncam
        H(:,:,cc) = camstruct_adjusted(cams(cc)).H;
    end
    
    CFPlot(H, 0.1)
    title('Extrinics Before and After Loading Previous Bundle Adjustment')
    axis equal
    %cd(default_dir)
    
    if options.ba ==1
        return
    else
        camstruct = camstruct_adjusted;
    end
end

npts = length(options.ba_pts);
nsteps = length(options.ba_tsteps);

stereo_tri = StereoTriangulation_svob(camstruct, options);
X_orig = zeros(3*npts,nsteps,length(stereo_tri));
for pair = 1:length(stereo_tri)
    X_orig(:,:,pair) = reshape(stereo_tri(pair).pts(:,options.ba_tsteps,options.ba_pts),[],length(options.ba_tsteps)); 
end
X_mean = nanmean(X_orig,3);

delta_vec_0 = reshape(reshape(X_mean,[],length(options.ba_tsteps)),[],1);
delta_vec_0 = [delta_vec_0; zeros(3*2*ncam,1)];

%Set options of fminsearch to plot function evaluations.
options_fmin = optimset('PlotFcns', @optimplotfunccount, 'Algorithm', 'levenberg-marquardt', 'MaxFunEvals', 20000, 'MaxIter', 1000);
%find the parameters that minimize error
delta_vec = lsqnonlin(@(delta_vec) fcost2(camstruct, options, delta_vec), delta_vec_0, [],[],options_fmin);

%Reset extrinsics to new values
param = delta_vec(3*npts*nsteps+1:end);
delta_vec_rot = param(1:length(param)/2);
delta_vec_pos = delta_vec(length(param)/2+1:length(param));
%delta_vec_rot = delta_vec;
%delta_vec_pos = zeros(length(delta_vec_rot),1);

camstruct_adjusted = camstruct;
for cc = 1:ncam
    camstruct_adjusted(cams(cc)).H = camstruct_adjusted(cams(cc)).H*[caltech_rodrigues(delta_vec_rot(3*(cc-1)+1:3*cc,1)),delta_vec_pos(3*(cc-1)+1:3*cc,1);zeros(1,3),1];
    Hin = invH(camstruct_adjusted(cams(cc)).H);
    camstruct_adjusted(cams(cc)).T = Hin(1:3,4);
    camstruct_adjusted(cams(cc)).om = caltech_rodrigues(Hin(1:3,1:3));
end
camstruct_exp = camstruct_adjusted;
save(savefile, 'camstruct_exp')
%cd(default_dir);

%Compile New Extrisics and Show on Same Plot
for cc = 1:ncam
    H(:,:,cc) = camstruct_adjusted(cams(cc)).H;
end
figure
X_new = reshape(delta_vec(1:3*npts*nsteps),3,[]);
plot3(X_new(1,:)',X_new(2,:)',X_new(3,:)','*b');

CFPlot(H, 0.1)
title('Extrinics Before and After Adjustement')
axis equal

