function [camstruct,options] = Init_Flight_MotionEst_Synth(camstruct,options)
%% --------------------------Set Options----------------------------------
%Import Options
options.groups          = [1];
options.link_names      = {'Body','Humerus','Radius', 'Metacarpal 3', 'Metacarpal 4','Metacarpal 5'};
options.dof_names       = {'X', 'Y', 'Z', '\theta_x', '\theta_y','\theta_z','\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6','\theta_7','\theta_8','\theta_9','\theta_{10}'};
options.tstart          = 1;                  %Note: due to sync delay the first 
options.tstop           = 400;                  %Useable timestep will be tstart+1 
options.interp          = 1;                    %1- data Was NOT interpolated, 0- otherwise;
options.dt              = 10;

options.plotflag        = 0;
options.path            = 'C:\Synthetic2016';
options.default_dir     = pwd;
options.fs              = options.fs_c;

%Trajectory Estimation Options
options.est.cams            = [1:10];
options.est.groups          = options.groups;
options.est.tstart          = 1;
options.est.tstop           = options.tstop - options.tstart+1;
options.est.state_init      = [1.0e+03 *[1.2500,1.5000,1.0000],0,pi/8,pi/2,...
                                130/180*pi-20/180*pi,-90/180*pi-30/180*pi,-90*pi/180,...
                                pi/4,...
                                10/180*pi,pi/8,...
                                60/180*pi,pi/8,...
                                100*pi/180,pi/8]';
%Plot Options
options.plot.pts           = [1:5,6,8,10,12,14];
options.plot.pts_orig      = [1:5,6,8,10,12,14];
options.plot.reprojframe   = 405;
options.plot.tstart        = 1;
options.plot.tstop         = (options.tstop - options.tstart)-(options.plot.tstart-1);
options.plot.linespec1        = {'.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g','^-r','^-b','^-g', '^-m','^-k','^-c','^--r','^--b','^--g'};
options.plot.linespec2        = {'+-r','+-b','+-g', '+-m','+-k','+-c','+--r','+--b','+--g','o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g'};
options.plot.linespec3        = {'o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g','.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g'};
options.plot.colors         =  {'r', 'g', 'b', 'c', 'm', 'k'};
options.plot.colors2        = [255,255,255,128,0,0,0,0,0,128,255,255;0,128,255,255,255,255,255,128,0,0,0,0;0,0,0,0,0,128,255,255,255,255,255,128]'/255;
options.plot.savepath       = 'C:\ShandongData2015\Batflight_07242015\Synthetic';
options.plot.savefig        = 0;
options.plot.saveim_reproj  = 0;
options.plot.saveim_reproje = 0;
options.plot.fig_txt_props  = {'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'Bold'};

%% Define the Skeleton
SkeletonDefn3
links        = get_group_links(synthConfig.link,options.groups);
options.link = synthConfig.link(links);
options      = create_state_vec(options);
options      = create_meas_vec(options);

%% Set the point associations and create a matrix of camera measurements
for cc = options.est.cams
    %camstruct(cc).pt_assoc = {[1,2,3,4,5],[6,3],[8,6],[10,8],[12,8],[14,8]};
    camstruct(cc).pt_assoc = {[1,2,3,4,5],[6],[8],[10],[12],[14]};
end
options.est.meas = create_meas_matrix(camstruct, options);

%% Filter - EKF
%State matrix with all points
options.est.x_km1 = options.est.state_init;

%Covariance Matrix for each point
link_list = get_group_links(options.link, options.est.groups);
options.est.Sigma_k = calc_Rt_joint(link_list, options.link);

% Motion model: Just kinematic point-mass model.  xn = x + u*delta_t;
options.est.state_update_model = @(x_km1, x_km2, links, Rt_handle) mm_ConstVel(x_km1, x_km2, options, links, Rt_handle);
options.est.Rt_handle          = @(ll)        calc_Rt_joint(ll,options.link);

% Measurement model: Each measurment is a col-vector of length 3*n where n
%   is the number of fixed markers on the bat body.  Assume exact knowledge
%   of the marker positions in body-frame (comes from param.mkr) and exact
%   orientation of the body in 3D space (comes from the upper-left of the H
%   matrix at every timestep)
options.est.msmt_model = @(x, t) CamNet_JS(x, camstruct, options);








