function [camstruct,options] = Init_Flight_Pts_20150724_test004(camstruct,options)

%% --------------------------Set Options----------------------------------
%Import Options
options.groups          = [1];
options.pts             = [4];
options.pt_names        = {'body', 'rshoulder','elbow','wrist','digit3','digit4'};
options.tstart          = 390;                  %Note: due to sync delay the first 
options.tstop           = 425;                  %Useable timestep will be tstart+1 
options.interp          = 0;                    %1- data Was NOT interpolated, 0- otherwise;
options.cams            = [301,302,303,310,312,318,320,325,333];
options.plotflag        = 0;
options.path            = 'F:\ShandongData2014\batflight_2014_07_16\video_002\run2_r2r3';
options.default_dir     = pwd;
options.fs              = 120;


%Trajectory Estimation Options
options.est.pts             = options.pts;
options.est.tstart          = 1;
options.est.tstop           = options.tstop - options.tstart+1;
options.est.state_init      = repmat([-1.091;-0.25;-0.290],length(options.pts),1);

%Plot Options
options.plot.pts            = options.pts;
options.plot.reprojframe    = 400;
options.plot.tstart         = 5;
options.plot.tstop          = (options.tstop - options.tstart)-(options.plot.tstart-1);
options.plot.linespec1     = {'.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g','+-r','+-b','+-g', '+-m','+-k','+-c','+--r','+--b','+--g'};
options.plot.linespec2     = {'+-r','+-b','+-g', '+-m','+-k','+-c','+--r','+--b','+--g','o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g'};
options.plot.linespec3     = {'o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g','.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g'};
options.plot.colors         =  {'r', 'g', 'b', 'c', 'm', 'k'};
options.plot.colors2        = [255,255,255,128,0,0,0,0,0,128,255,255;0,128,255,255,255,255,255,128,0,0,0,0;0,0,0,0,0,128,255,255,255,255,255,128]'/255;
options.plot.savepath       = 'D:\Users\Matt\Documents\VT\Research\Motion_estimation_dev\Papers_and_Presentations\Bender2015scitech\';
options.plot.savefig        = 0;
options.plot.saveim_reproj  = 0;
options.plot.saveim_reproje = 0;
options.plot.fig_txt_props  = {'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'Bold'};

%% Set the point associations and create a matrix of camera measurements
ncam = length(options.cams);
cams = options.cams;
npts = length(options.pts);
options.est.meas = zeros(ncam*npts*2,options.tstop-options.tstart+1);
for cc = 1:ncam
    time_vect = [options.tstart:options.tstop]-camstruct(cams(cc)).start_frame+1+floor(options.fs*camstruct(cams(cc)).sync_del);
    for pp = 1:npts
        options.est.meas (2*npts*(cc-1)+2*(pp-1)+1:2*npts*(cc-1)+2*pp,:) = ...
            camstruct(cams(cc)).pts_sync(:,time_vect,options.pts(pp));
    end
end
options.nstate = 3*npts;

%% Filter - EKF
%State matrix with all points
options.est.x_km1 = options.est.state_init;

%Covariance Matrix for each point
Rk = calc_Rt();
Rk_cell = repmat({Rk},1,length(options.pts));
options.est.Sigma_k = blkdiag(Rk_cell{:});


% Motion model: Just kinematic point-mass model.  xn = x + u*delta_t;
t = 0;
u_k = 0;
options.est.Rt_handle          = @()        calc_Rt();
options.est.state_update_model = @(x_km1, Rt_handle) ZeroDyn(x_km1, u_k, t, Rt_handle);

% Measurement model: Each measurment is a col-vector of length 3*n where n
%   is the number of fixed markers on the bat body.  Assume exact knowledge
%   of the marker positions in body-frame (comes from param.mkr) and exact
%   orientation of the body in 3D space (comes from the upper-left of the H
%   matrix at every timestep)
options.est.msmt_model = @(x, t) CamNet(x, camstruct(options.cams));