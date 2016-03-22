function [camstruct,options] = Init_Flight_BatFlight_20150724_test004_wrist(camstruct,options)
%% --------------------------Set Options----------------------------------
%Import Options
options.groups          = [1,2,3];
options.link_names      = {'Body','Humerus','Radius','Wrist', 'Metacarpal 3', 'Metacarpal 4','Metacarpal 5'};
options.dof_names        = {'X', 'Y', 'Z', '\theta_x', '\theta_y','\theta_z','\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6','\theta_7','\theta_8','\theta_9','\theta_{10}','\theta_{11}'};
options.tstart          = 390;                  %Note: due to sync delay the first 
options.tstop           = 425;                  %Useable timestep will be tstart+1 
options.interp          = 1;                    %1- data Was NOT interpolated, 0- otherwise;

options.plotflag        = 0;
options.path            = 'C:\ShandongData2015\Batflight_07242015\Test004';
options.default_dir     = pwd;
options.fs              = options.fs_c;

%Trajectory Estimation Options
options.est.cams        = [301,302,303,310,312,318,320,325,333];
options.est.groups          = options.groups;
options.est.tstart          = 1;
options.est.tstop           = options.tstop - options.tstart+1;
options.est.state_init      = [-1.093,-0.2433,-0.2678,90*pi/180,-100*pi/180,180*pi/180,...%]';%,...
                                -270*pi/180,-pi/2,-60*pi/180,...
                                30/180*pi...
                                -75*pi/180,...
                                55*pi/180,20*pi/180,...
                                pi/2,20*pi/180,...
                                100/180*pi,-20*pi/180]';

%Plot Options
options.plot.pts           = [1,2,3,4,6,7,11,12,14,15,17];
options.plot.pts_orig      = [4,1,5,6,8,7,10,9,14,13,17];
options.plot.reprojframe   = 405;
options.plot.tstart        = 6;
options.plot.tstop         = (options.tstop - options.tstart)-(options.plot.tstart-1);
options.plot.linespec1        = {'.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g','^-r','^-b','^-g', '^-m','^-k','^-c','^--r','^--b','^--g'};
options.plot.linespec2        = {'+-r','+-b','+-g', '+-m','+-k','+-c','+--r','+--b','+--g','o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g'};
options.plot.linespec3        = {'o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g','.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g'};
options.plot.colors         =  {'r', 'g', 'b', 'c', 'm', 'k'};
options.plot.colors2        = [255,255,255,128,0,0,0,0,0,128,255,255;0,128,255,255,255,255,255,128,0,0,0,0;0,0,0,0,0,128,255,255,255,255,255,128]'/255;
options.plot.savepath       = 'C:\Users\Matt\Documents\GitHub\SciTechPaper';
options.plot.savefig        = 0;
options.plot.saveim_reproj  = 0;
options.plot.saveim_reproje = 0;
options.plot.fig_txt_props  = {'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'Bold'};

%% Define the Skeleton
SkeletonDefn_BatFlight_20150724_test004_wrist
links        = get_group_links(synthConfig.link,options.groups);
options.link = synthConfig.link(links);
options      = create_state_vec(options);
options      = create_meas_vec(options);

%% Set the point associations and create a matrix of camera measurements
for cc = options.est.cams
    camstruct(cc).pt_assoc = {[4,1,5],[6,5],[8,7,6],[8,8],[10,9,8],[14,13,8],[17,8]};
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








