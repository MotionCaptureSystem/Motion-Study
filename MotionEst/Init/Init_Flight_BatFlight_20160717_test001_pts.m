function [camstruct,options] = Init_Flight_BatFlight_20160717_test001_pts(camstruct,options)
%% --------------------------Set Options----------------------------------
%Import Options
options.est.type        = 'point';
options.pts             = [105,141,100,93,87,89,91,46,54,49,56,44,58];
options.groups          = [1];
%options.link_names      = {'Body','Humerus','Radius','Wrist', 'Metacarpal 3', 'Metacarpal 4','Metacarpal 5'};
%options.dof_names       = {'X', 'Y', 'Z', '\theta_x', '\theta_y','\theta_z','\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6','\theta_7','\theta_8','\theta_9','\theta_{10}','\theta_{11}'};
options.tstart          = 368;                  %Note: due to sync delay the first 
options.tstop           = 452;                  %Useable timestep will be tstart+1 
options.dt              = 1;
options.interp          = 1;                    %1- data Was NOT interpolated, 0- otherwise;

options.plotflag        = 0;
%options.path           = 'C:\ShandongData2015\Batflight_07242015\Test004';
options.default_dir     = pwd;
options.fs              = options.fs_c;

%Trajectory Estimation Options
options.est.cams            = [301,302,306,309:314,318,319,322,324,327,328,331,334,337,338,340,341];
options.est.tstart          = 1;
options.est.tstop           = options.tstop - options.tstart+1;
options.est.state_init      = zeros(3*length(options.pts),1);

%Plot Options
options.plot.pts            = [1,2,3,4,6,7,11,12,14,15,17];
options.plot.pts_orig       = [105,141,100,93,87,89,91,46,54,49,56,44,58];
options.plot.reprojframe    = 405;
options.plot.tstart         = 6;
options.plot.tstop          = (options.tstop - options.tstart)-(options.plot.tstart-1);
options.plot.linespec1      = {'.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g','^-r','^-b','^-g', '^-m','^-k','^-c','^--r','^--b','^--g'};
options.plot.linespec2      = {'+-r','+-b','+-g', '+-m','+-k','+-c','+--r','+--b','+--g','o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g'};
options.plot.linespec3      = {'o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g','.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g'};
options.plot.colors         = {'r', 'g', 'b', 'c', 'm', 'k'};
options.plot.colors2        = [255,255,255,128,0,0,0,0,0,128,255,255;0,128,255,255,255,255,255,128,0,0,0,0;0,0,0,0,0,128,255,255,255,255,255,128]'/255;
options.plot.savepath       = 'C:\Users\Matt\Documents\GitHub\SciTechPaper';
options.plot.savefig        = 0;
options.plot.saveim_reproj  = 0;
options.plot.saveim_reproje = 0;
options.plot.fig_txt_props  = {'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'Bold'};

%% Define the Skeleton
%SkeletonDefn_BatFlight_20160717_test001
%links        = get_group_links(synthConfig.link,options.groups);
%options.link = synthConfig.link(links);
%options      = create_state_vec(options);
%options      = create_meas_vec(options);

%% Set the point associations and create a matrix of camera measurements
% for cc = options.est.cams
%     camstruct(cc).pt_assoc = {[105,141,100],[93],[87,89,91],[],[46,54],[49,56],[44,58]};
% end
options.est.meas = create_meas_matrix(camstruct, options);

%% Filter - EKF
%State matrix with all points
options.est.x_km1 = options.est.state_init;

%Covariance Matrix for each point
%link_list = get_group_links(options.link, options.est.groups);
options.est.Sigma_k = kron(eye(length(options.pts)),calc_Rt());
u_k = 0;
t = 0;

% Motion model: Just kinematic point-mass model.  xn = x + u*delta_t;
options.est.Rt_handle          = @()        calc_Rt();
options.est.state_update_model = @(x_km1, hist, Rt_handle) ZeroDyn(x_km1,u_k,t, Rt_handle);

% Measurement model: Each measurment is a col-vector of length 3*n where n
%   is the number of fixed markers on the bat body.  Assume exact knowledge
%   of the marker positions in body-frame (comes from param.mkr) and exact
%   orientation of the body in 3D space (comes from the upper-left of the H
%   matrix at every timestep)
options.est.msmt_model = @(x, t) CamNet(x, camstruct);








