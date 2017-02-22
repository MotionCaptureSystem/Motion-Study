function [camstruct,options] = Init_Flight_BatFlight_20160717_test001_flexBB(camstruct,options)
%% --------------------------Set Options----------------------------------
%Import Options
options.est.type        = 'joint';
options.groups          = [1,2,3];
options.link_names      = {'Body','Body Flex','Humerus','Radius','Wrist', 'Metacarpal 3', 'Metacarpal 4','Metacarpal 5'};
options.dof_names       = {'X', 'Y', 'Z', '\theta_x', '\theta_y','\theta_z','\theta_b','\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6','\theta_7','\theta_8','\theta_9','\theta_{10}','\theta_{11}'};
options.tstart          = 377;                  %Note: due to sync delay the first 
options.tstop           = 387;                  %Useable timestep will be tstart+1 
options.dt              = 1;
options.interp          = 1;                    %1- data Was NOT interpolated, 0- otherwise;

options.plotflag        = 0;
%options.path           = 'C:\ShandongData2015\Batflight_07242015\Test004';
options.default_dir     = pwd;
options.fs              = options.fs_c;

%Trajectory Estimation Options
options.est.cams            = [301,302,306,309:314,318,319,322,324,327,328,331,334,337,338,340,341];
options.est.groups          = options.groups;
options.est.tstart          = 1;
options.est.tstop           = options.tstop - options.tstart+1;
options.est.state_init      = [-38.7,-234.6,13.12,...
                               201.9*pi/180,12.83*pi/180,10.84*pi/180,...
                               55.53*pi/180,...
                               75.49*pi/180,62.29*pi/180,-96.19*pi/180,...
                               -19.4*pi/180,...
                               -8.48*pi/180,...
                               -28.1*pi/180,48.39*pi/180,...
                               -35.25*pi/180,75.33*pi/180,...
                               23.56*pi/180,37.5*pi/180]';

%Plot Options
options.plot.pts           = [[1,2],[3,4,6,7,11,12,14,15,17]+1];
options.plot.pts_orig      = [141,105,100,93,87,89,91,46,54,49,56,44,58];
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
SkeletonDefn_BatFlight_20160717_test001_flexBB
links        = get_group_links(synthConfig.link,options.groups);
options.link = synthConfig.link(links);
options      = create_state_vec(options);
options      = create_meas_vec(options);

%% Set the point associations and create a matrix of camera measurements
for cc = options.est.cams
    camstruct(cc).pt_assoc = {[105,141], [100], [93],[87,89,91],[],[46,54],[49,56],[44,58]};
end
options.est.meas = create_meas_matrix(camstruct, options);

%% Filter - EKF
%State matrix with all points
options.est.x_km1 = options.est.state_init;

%Covariance Matrix for each point
link_list = get_group_links(options.link, options.est.groups);
options.est.Sigma_k = calc_Rt_joint(link_list, options.link);

% Motion model: Just kinematic point-mass model.  xn = x + u*delta_t;
options.est.state_update_model = @(x_km1, x_km2, links, Rt_handle) mm_ConstVel_BBTrans(x_km1, x_km2, options, links, Rt_handle);
options.est.Rt_handle          = @(ll)        calc_Rt_joint(ll,options.link);

% Measurement model: Each measurment is a col-vector of length 3*n where n
%   is the number of fixed markers on the bat body.  Assume exact knowledge
%   of the marker positions in body-frame (comes from param.mkr) and exact
%   orientation of the body in 3D space (comes from the upper-left of the H
%   matrix at every timestep)
options.est.msmt_model = @(x, t) CamNet_JS(x, camstruct, options);








