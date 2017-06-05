%% MOTIONSTUDY_SCRIPT.m
clear all
close all

addpath('Calibration', 'Common', 'FeatIdent','ImageProc','MotionEst','Results','PtsInfo','CFDPrep')
addpath(['.',filesep,'MotionEst',filesep,'Init'],...
        ['.',filesep,'MotionEst',filesep,'Models'],...
        ['.',filesep,'MotionEst',filesep,'Stereo'],...
        ['.',filesep,'MotionEst',filesep,'TrajEst'],...
        ['.',filesep,'MotionEst',filesep,'Manifolds'],...
        ['.',filesep,'MotionEst',filesep,'Manifolds',filesep,'GPDM'],...
        ['.',filesep,'MotionEst',filesep,'Tracking'])
%% Load the Camera Measurements
options.path = 'C:\ShandongData2016\BatFlight_20160717\Test001';
a = load([options.path,filesep,'CamStruct.mat']);

Cam(a.cams(1):a.cams(2)) = a.Cam;
clear a

%% Load the Calibration
fprintf('Loading Calibration Parameters ...\n')
options.cams = [301, 302, 306, 309, 310, 311, 312, 313, 314, 318, 319, 322, 324, 327, 328, 331, 334, 337, 338, 340, 341];
%cal_tech intrinsics
%Cam = load_caltech_intrinsic(Cam, options, options.cams);
%Svoboda Extrinsics
options.cams_im2pts = options.cams;
options.cams_cal    = options.cams;
% [options, Cam] = load_svoboda_cal5(Cam,options);
options.ucs_size = norm(Cam(options.cams(1)).H(1:3))/100;

%% Rectify and Synchronise Points
Cam = rectify(Cam,options.cams);
options.fs_c = 120;
Cam = sync_cams(Cam);
%% Perform Stereo Triangulation
options.stereo.cams = options.cams;
options.stereo.pts = [1:250];
options.stereo.tstart = 368;
options.stereo.dt = 1;
options.stereo.tstop = 452;
options.plot.colors = hsv(length(options.stereo.pts));

fprintf('Running Stereo Triangulation ... \n')
Stereo = StereoTriangulation_svob(Cam, options);
save([options.path,filesep,'StereoStruct.mat'],'Stereo')

%% Load Initialization Options
%[Cam,options] = Init_Flight_BatFlight_20160717_test001_flexBB(Cam,options);
options.est.type        = 'joint';
options.groups          = [1];
options.link_names      = {'Body Link 1','Body Yaw','Body Flex','R Humerus','R Radius','L Humerus','L Radius',...%'R Wrist', ...
                           'R Metacarpal 3', 'R Metacarpal 4','R Metacarpal 5',...%'L Wrist',
                           'L Metacarpal 3', 'L Metacarpal 4','L Metacarpal 5'...
                           'LD3Phal1', 'LD3Phal2', 'LD4Phal1', 'LD4Phal2', 'LD5Phal1','LD5Phal2',...
                           'RD3Phal1', 'RD3Phal2', 'RD4Phal1', 'RD4Phal2', 'RD5Phal1','RD5Phal2'};
options.dof_names       = {'X', 'Y', 'Z', '\theta_r', '\theta_p','\theta_y','\theta_b','\theta_1','\theta_2','\theta_3','\theta_4','\theta_5',...
                            '\theta_6','\theta_7','\theta_8','\theta_9','\theta_{10}','\theta_{11}','\theta_{12}',...
                            '\theta_{13}','\theta_{14}','\theta_{15}','\theta_{16}','\theta_{17}','\theta_{18}','\theta_{19}','\theta_{20}','\theta_{21}','\theta_{22}'...
                            '\theta_{23}','\theta_{24}','\theta_{25}','\theta_{26}','\theta_{27}','\theta_{28}',...
                            '\theta_{29}','\theta_{30}','\theta_{31}','\theta_{32}','\theta_{33}','\theta_{34}',...
                            '\theta_{35}','\theta_{36}','\theta_{37}','\theta_{38}','\theta_{39}','\theta_{40}',...
                            '\theta_{41}','\theta_{42}','\theta_{43}','\theta_{44}','\theta_{45}','\theta_{46}','\theta_{47}','\theta_{48}','\theta_{49}','\theta_{50}'};
                        
options.tstart          = 368;       %Note: due to sync delay the first 
options.tstop           = 450;       %Useable timestep will be tstart+1 
options.dt              = 1;
options.interp          = 1;         %1- data Was NOT interpolated, 0- otherwise;

options.plotflag        = 0;
options.default_dir     = pwd;
options.fs              = options.fs_c;

%Trajectory Estimation Options
options.est.cams            = [301,302,310:314,318,319,322,324,327,328,331,337,338,340];%[301,302,306,309:314,318,319,322,324,327,328,331,334,337,338,340,341];
options.est.groups          = options.groups;
options.est.tstart          = 1;
options.est.tstop           = options.tstop - options.tstart+1;

% Define the Skeleton
root_name = 'SkeletonDefn_BatFlight_20160717_test001_';
kindef = {'noWrist',...%'flexBB_both_wings',...'2dofPhal',...
          'manual',...%'2dofPhal_noWrist',...
          '2dofPhal_noWrist2',...
          '1dofPhal',...
          '1dofPhal2_noWrist'};
%% 
for ee = 2%1:length(kindef)
    clear synthConfig
    eval([root_name,kindef{ee}])
    links        = get_group_links(synthConfig.link,options.groups);
    options.link = synthConfig.link(links);
    options      = create_state_vec(options);
    options      = create_meas_vec(options);

    % Set the point associations and create a matrix of camera measurements
    for ll = 1:length(synthConfig.link)
        if ll == 1
            pt_assoc{ll} = synthConfig.link(ll).pt_nums;
            continue;
        end
        if length(unique(synthConfig.link(ll).pt_nums))~=length(synthConfig.link(ll).pt_nums) 
            pt_assoc{ll} = [];
        else
            pt_assoc{ll} = synthConfig.link(ll).pt_nums(1:end-1);
        end
    end
    
    for cc = options.est.cams
        Cam(cc).pt_assoc = {};
        %Cam(cc).pt_assoc = {[105,141], [100], [93],[87,89,91],[],[46],[41,56],[44,58]};
        Cam(cc).pt_assoc = pt_assoc;
    end
    options.est.meas = create_meas_matrix(Cam, options);

    %Plot Options
    options.plot.pts_orig       = [pt_assoc{links}];
    options.plot.pts            = [pt_assoc{links}];
    options.plot.reprojframe    = 400;
    options.plot.tstart         = 1;
    options.plot.tstop          = (options.tstop - options.tstart)-(options.plot.tstart-1);
    options.plot.linespec1      = {'.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g','^-r','^-b','^-g', '^-m','^-k','^-c','^--r','^--b','^--g'};
    options.plot.linespec2      = {'+-r','+-b','+-g', '+-m','+-k','+-c','+--r','+--b','+--g','o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g'};
    options.plot.linespec3      = {'o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g','.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g'};
    options.plot.colors         =  hsv(length(options.plot.pts));
    options.plot.savepath       = 'C:\Users\Matt\Documents\GitHub\SciTechPaper';
    options.plot.savefig        = 0;
    options.plot.saveim_reproj  = 0;
    options.plot.saveim_reproje = 0;
    options.plot.fig_txt_props  = {'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'Bold'};

    start_pts = [];
    for ss = 1:length(Stereo)
        if ~isnan(Stereo(ss).pts(1,2,options.plot.pts_orig(1)))
            start_pts = [start_pts, Stereo(ss).pts(:,2,options.plot.pts_orig(1))];
        end
    end
    sum_of_start = nanmedian(start_pts,2);
    body_arm_init      = [0,0,0,...
                           0,0*pi/180,...
                           20*pi/180,...
                           -20*pi/180,...
                           90*pi/180,   0*pi/180,  200*pi/180,... %right arm
                           0,...  
                           -90*pi/180, 0*pi/180, 120*pi/180,...   %left arm
                           0];%,   0, 0,...%right hand
    options.est.state_init = [body_arm_init,zeros(1,sum([options.link.nDof])-length(body_arm_init))]';
    %                                0,   0,0,...
    %                                0,   0,0,...
    %                                0,   0,0,...%left hand
    %                                0,   0,0,...
    %                                0,   0,0,...
    %                                60*pi/180,0,0,...
    %                                90*pi/180,0,0,...
    %                                40*pi/180,0,0,...
    %                                80*pi/180,0,0,...
    %                                80*pi/180,0,0,...
    %                                40*pi/180,0,0]';

    % Filter Options - UKF
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
    options.est.msmt_model = @(x, t) CamNet_JS(x, Cam, options);
    %%Run the Estimator
    %[EstStruct, options] = traj_estimation(Cam(options.est.cams), options);

    nsteps = options.est.tstop-options.est.tstart+options.interp;
    % Will the estimator correspond features?
    y = 'n';
    fprintf('Trajectory Estimation is Running .... \n')
    tic

    if strcmpi(y,'n')
        % Run UKF 
        %[X_ekf, Sig_X_ekf, mat_mags_ekf] = run_extended_kf(x_km1, Sigma_k, zeros(3*npts,nsteps), meas, state_update_model, state_jac, msmt_model, msmt_jac, Rt_handle);

        [X_ukf, Sig_X_ukf]      = run_unscented_kf_recursive_condind(options.est.x_km1, options.est.Sigma_k, zeros(options.nstate,nsteps), ...
                                                                       options.est.meas, options.est.state_update_model, options.est.msmt_model, ...
                                                                       options.est.Rt_handle, options);
                                                                   
%         [X_ukf, Sig_X_ukf]      = run_unscented_kf_recursive2(options.est.x_km1, options.est.Sigma_k, zeros(options.nstate,nsteps), ...
%                                                                    options.est.meas, options.est.state_update_model, options.est.msmt_model, ...
%                                                                    options.est.Rt_handle, options);
    else
        [X_ukf, Sig_X_ukf, meas] = run_unscented_kf_track2(options.est.x_km1, options.est.Sigma_k, zeros(options.nstate,nsteps), ...
                                                                       options.est.meas, options.est.state_update_model, options.est.msmt_model, ...
                                                                       options.est.Rt_handle, options, Cam(options.est.cams));
        %eststruct.n_correct = meas.n_correct;
        eststruct.z_gg_auto = meas.z_auto_corr;
    end
    %outstruct.ekf.X         = X_ekf;
    eststruct.ukf.X         = X_ukf;
    %outstruct.ekf.Sig_X     = Sig_X_ekf;
    eststruct.ukf.Sig_X     = Sig_X_ukf;
    toc
    fprintf('Trajectory Estimation is Complete .... \n')

    if strcmp(options.est.type,'joint')
        q      = eststruct.ukf.X;
        nsteps = size(q,2);
        %create the body basis transforms and the coordinate transforms for each
        %time step2
        links = get_group_links(options.link,options.groups);
        dof = 0;
        for ll = links
            nDof = options.link(ll).nDof;
            dof = dof+options.link(ll).nDof;
            for kk = 1:nsteps
                q_lk = q(dof-nDof+1:dof,kk);
                if strcmp(options.link(ll).IDkern,'DH')
                    tDof = options.link(ll).tDof';
                    kinc(kk).link(ll).H=DHTransforms((~tDof).*q_lk+options.link(ll).thetas,...
                                options.link(ll).alphas,... %0*(tDof).*q_lk + options.link(ll).disps,...
                                (tDof).*q_lk + options.link(ll).disps,...
                                options.link(ll).offsets);
                elseif  strcmp(options.link(ll).IDkern,'YPR')
                    theta = q_lk(4:6);
                    d = q_lk(1:3);
                    kinc(kk).link(ll).H = YPRTransform(theta, d);
                end
            end
        end

        npts = size([options.link.BFvecs],2);
        features = zeros(3*npts,nsteps);
        n_pts_tot = 0;
        for ll = links
            n_bf_pts = size(options.link(ll).BFvecs,2);
            n_pts_tot = n_pts_tot+n_bf_pts;
            if n_bf_pts
                for kk = 1:nsteps
                X = [eye(3,3),zeros(3,1)]*hnode2node(kinc(kk),options,1,ll)*[options.link(ll).BFvecs;ones(1,n_bf_pts)];
                features(3*(n_pts_tot-n_bf_pts)+1:3*n_pts_tot,kk) = X(:);
                end
                %assoc(:,(n_pts_tot-n_bf_pts)+1:n_pts_tot) = [ll*ones(1,n_bf_pts);1:n_bf_pts];
            end
        end
        eststruct.ukf.Features = features;
        eststruct.kinc = kinc;

    elseif strcmp(options.est.type,'point')
        eststruct.ukf.Features = X_ukf;
    end

    save([kindef{ee},'.mat'],'-v7.3')
    fclose('all');
end
%% Plot State Results
plot_states(Cam,eststruct,options)
clear M
figure
for kk = 1:length(eststruct.kinc)
    clf
    plot_kin_chain(eststruct.kinc(kk),options,1,1)
    axis([-100,2000,-300,300,-300,200])
    %axis([-100,150,-500,500,-150,150])
    view([-129.2,38.4])
    %view([-108.1,49.9])
    grid on
    set(gca,options.plot.fig_txt_props{:})
    xlabel('x (mm)',options.plot.fig_txt_props{:})
    ylabel('y (mm)',options.plot.fig_txt_props{:})
    zlabel('z (mm)',options.plot.fig_txt_props{:})
    drawnow
    M(kk) = getframe(gcf);
end

vidObj = VideoWriter('FlyingBat.avi');
vidObj.FrameRate = 10;
open(vidObj);
writeVideo(vidObj,M);
close(vidObj);


%% Reprojection
reproj_error(Cam, eststruct, options)

%% Check for Correct Correspondences
if isfield(eststruct,'n_correct')
    n_correct = eststruct.n_correct;
    cams = options.est.cams;
    pts = [Cam(301).pt_assoc{:}];
    t_start = options.tstart;
    t_stop  = options.tstop;

    ncam = size(n_correct,1);
    colors = hsv(ncam);

    time = [1:size(n_correct,2)-2]';
    figure
    for cc = 1:ncam
        pts_k_nan = ~isnan(Cam(cams(cc)).pts(1,[t_start:t_stop]-Cam(cams(cc)).start_frame+1,pts));
        pts_km1_nan = ~isnan(Cam(cams(cc)).pts(1,[t_start:t_stop]-Cam(cams(cc)).start_frame,pts));
        corr_mat = pts_k_nan & pts_km1_nan;%& pts_km1_nan;
        corr_max = sum(corr_mat,3);
        %nan_steps = sum(isnan(Cam(cams(cc)).pts(1,[t_start:t_stop]-Cam(cams(cc)).start_frame+1,pts)),3)>=length(pts);
        subplotfill(3,7,cc)
        hold on
        %plot(time,nan_steps','o','color',colors(cc,:))
        bar(time,[n_correct(cc,3:end)',corr_max'-n_correct(cc,3:end)'],'stacked');
        %bar(time,[sum(pts_k_nan,3)',corr_max'-sum(pts_k_nan,3)'],'stacked');
        axis tight
        h(cc) = gca;
        percent_corr(cc) = sum(n_correct(cc,3:end))/sum(corr_max)*100;
        title(['Cam ', num2str(cc), ' - corr %', num2str(percent_corr(cc))],'FontName','Times New Roman','FontSize',12)
        grid on
        if ismember(cc,[1,8,15])
            ylabel('N Correct','FontName','Times New Roman','FontSize',12)
        end
        if ismember(cc,[15:21])
            xlabel('Frame','FontName','Times New Roman','FontSize',12)
        end
        if ismember(cc,[1:14])
            set(gca,'XTickLabel',[]);
        end
        if ~ismember(cc,[1,8,15])
            set(gca,'YTickLabel',[])
        end
    end

    ymax = 0; 
    for cc = 1:ncam
        ylim = get(h(cc),'Ylim');
        if ymax<ylim(2)
            ymax = ylim(2);
        end
    end

    for cc = 1:ncam
    set(h(cc),'Ylim',[0,ymax])
    end

    figure
    H = reshape([Cam.H],4,4,[]);
    array_center = mean(H(1:3,4,:),3);
    mean_sub = zeros(4,4,ncam);
    mean_sub(1:3,4,:) = repmat(array_center,1,1,ncam);
    H_zmean = H-mean_sub;
    CFPlot(H_zmean,options.ucs_size);

    for cc = 1:ncam
       text(H_zmean(1,4,cc),H_zmean(2,4,cc),H_zmean(3,4,cc),sprintf('%3.0f',percent_corr(cc)),'FontName','Times New Roman','FontSize',12)
    end
end