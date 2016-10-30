function [camstruct,options] = Init_EmpPot_Sim1(camstruct,options)
%% --------------------------Set Options----------------------------------
%Import Options
options.est.type        = 'point';
options.groups          = [1];
options.pts             = 1;
options.nstate          = length(options.pts)*3;
options.link_names      = {'Body'};
options.dof_names       = {'q1', 'q2', 'q3'};
options.tstart          = 1;                    %Note: due to sync delay the first 
options.tstop           = 10;                 %Useable timestep will be tstart+1 
options.dt              = 1;
options.interp          = 1;                    %1- data Was NOT interpolated, 0- otherwise;
options.plotflag        = 0;
options.default_dir     = pwd;
options.fs              = options.fs_c;

%Trajectory Estimation Options
options.est.cams            = [1,2];
options.est.groups          = options.groups;
options.est.tstart          = 1;
options.est.tstop           = options.tstop - options.tstart+1;
options.est.state_init      = [.2236,.4472,.8660]';

%Plot Options
options.plot.pts            = [1];
options.plot.pts_orig       = [1];
options.plot.reprojframe    = 25;
options.plot.tstart         = 1;
options.plot.tstop          = (options.tstop - options.tstart)-(options.plot.tstart-1);
options.plot.linespec1      = {'.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g','^-r','^-b','^-g', '^-m','^-k','^-c','^--r','^--b','^--g'};
options.plot.linespec2      = {'+-r','+-b','+-g', '+-m','+-k','+-c','+--r','+--b','+--g','o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g'};
options.plot.linespec3      = {'o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g','.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g'};
options.plot.colors         =  {'r', 'g', 'b', 'c', 'm', 'k'};
options.plot.colors2        = [255,255,255,128,0,0,0,0,0,128,255,255;0,128,255,255,255,255,255,128,0,0,0,0;0,0,0,0,0,128,255,255,255,255,255,128]'/255;
options.plot.savepath       = 'D:\Users\Matt\Documents\GitHub\MotionStudy\MotionEst\Manifolds';
options.plot.savefig        = 0;
options.plot.saveim_reproj  = 0;
options.plot.saveim_reproje = 0;
options.plot.fig_txt_props  = {'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'Bold'};

%% Set the point associations and create a matrix of camera measurements
options.est.meas = create_meas_matrix(camstruct, options);

%% Filter Settings
%inital state
options.est.x_km1 = options.est.state_init;

%Covariance Matrix for each point
options.est.Sigma_k = calc_Rt();

% Motion model
load D:\Users\Matt\Documents\GitHub\MotionStudy\MotionEst\Manifolds\ManifoldPts.mat
Rt_handle               = @()  calc_Rt();
options.est.Rt_handle   = Rt_handle;
beta                    = 10;
K                       = kernelQ_DeVito_mat(points(:,:),beta);
dVzdx_handle            = @(x_km1)  dempot(x_km1,points(:,:),K);
options.empman.ep       = 0.01;
options.est.state_update_model = @(x_km1, x_km2, Rt_handle) manifold_dyn(x_km1, x_km2, Rt_handle, dVzdx_handle, options);

% Measurement model: Each measurment is a col-vector of length 3*n where n
%   is the number of fixed markers on the bat body.  Assume exact knowledge
%   of the marker positions in body-frame (comes from param.mkr) and exact
%   orientation of the body in 3D space (comes from the upper-left of the H
%   matrix at every timestep)
options.est.msmt_model = @(x, t) CamNet(x, camstruct);








