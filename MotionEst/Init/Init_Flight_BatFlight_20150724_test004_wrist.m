%clear all
%close all
%% --------------------------Set Options----------------------------------
%Import Options
options.groups          = [1,2,3];
options.link_names      = {'Body','Humerus','Radius','Wrist', 'Metacarpal 3', 'Metacarpal 4','Metacarpal 5'};
options.dof_names        = {'X', 'Y', 'Z', '\theta_x', '\theta_y','\theta_z','\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6','\theta_7','\theta_8','\theta_9','\theta_{10}','\theta_{11}'};
options.tstart          = 390;                  %Note: due to sync delay the first 
options.tstop           = 425;                  %Useable timestep will be tstart+1 
options.interp          = 1;                    %1- data Was NOT interpolated, 0- otherwise;
options.cams            = [301,302,303,310,312,318,320,325,333];
options.plotflag        = 0;
options.path            = 'D:\ShandongData2015\Batflight_07242015\Test004';
options.default_dir     = pwd;
options.fs              = 120;

%Trajectory Estimation Options
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
options.plot.reprojframe   = 405;
options.plot.tstart        = 6;
options.plot.tstop         = (options.tstop - options.tstart)-(options.plot.tstart-1);
options.plot.linespec1        = {'.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g','^-r','^-b','^-g', '^-m','^-k','^-c','^--r','^--b','^--g'};
options.plot.linespec2        = {'+-r','+-b','+-g', '+-m','+-k','+-c','+--r','+--b','+--g','o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g'};
options.plot.linespec3        = {'o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g','.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g'};
options.plot.colors         =  {'r', 'g', 'b', 'c', 'm', 'k'};
options.plot.colors2        = [255,255,255,128,0,0,0,0,0,128,255,255;0,128,255,255,255,255,255,128,0,0,0,0;0,0,0,0,0,128,255,255,255,255,255,128]'/255;
options.plot.savepath       = 'D:\Users\Matt\Documents\GitHub\SciTechPaper';
options.plot.savefig        = 1;
options.plot.saveim_reproj  = 1;
options.plot.saveim_reproje = 1;
options.plot.fig_txt_props  = {'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'Bold'};

%% Define the Skeleton
SkeletonDefn_BatFlight_20150724_test004_wrist
links        = get_group_links(synthConfig.link,options.groups);
options.link = synthConfig.link(links);
options      = create_state_vec(options);
options      = create_meas_vec(options);

%% Load The Camera Measurements
Cam = handles.Cam(options.cams);
ncam = length(Cam);

for cc = 1:ncam
    dt = Cam(cc).start_frame-1+floor(options.fs*Cam(cc).sync_del);
    Cam(cc).pts = [];
    Cam(cc).pts(:,:,1) = Cam(cc).pts_sync(:,options.tstart-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88):options.tstop-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88),4);
    Cam(cc).pts(:,:,2) = Cam(cc).pts_sync(:,options.tstart-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88):options.tstop-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88),1);
    Cam(cc).pts(:,:,3) = Cam(cc).pts_sync(:,options.tstart-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88):options.tstop-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88),5);

    Cam(cc).pts(:,:,4:5) = Cam(cc).pts_sync(:,options.tstart-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88):options.tstop-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88),[6:-1:5]);
    Cam(cc).pts(:,:,6:8) = Cam(cc).pts_sync(:,options.tstart-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88):options.tstop-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88),[8:-1:6]);
    Cam(cc).pts(:,:,9) = Cam(cc).pts_sync(:,options.tstart-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88):options.tstop-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88),8);
    Cam(cc).pts(:,:,10) = Cam(cc).pts_sync(:,options.tstart-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88):options.tstop-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88),8);

    Cam(cc).pts(:,:,11:13) = Cam(cc).pts_sync(:,options.tstart-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88):options.tstop-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88),[10:-1:8]);
    Cam(cc).pts(:,:,14:16) = Cam(cc).pts_sync(:,options.tstart-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88):options.tstop-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88),[14,13,8]);
    Cam(cc).pts(:,:,17:18) = Cam(cc).pts_sync(:,options.tstart-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88):options.tstop-Cam(cc).start_frame+1+floor(Cam(cc).sync_del*119.88),[17,8]);
    Cam(cc).pt_assoc = [1,1,1,2,2,3,3,3,4,4,5,5,5,6,6,6,7,7;
                        1,2,3,1,2,1,2,3,1,2,1,2,3,1,2,3,1,2];
end



