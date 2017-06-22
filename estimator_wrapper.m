function [eststruct, options, X_ukf] = estimator_wrapper(Cam, options)

eval([options.root_name,options.kindef{ee}])

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
                               90*pi/180,   0*pi/180,  -50*pi/180,... %right arm
                               0,...  
                               -90*pi/180, 0*pi/180, 50*pi/180,...   %left arm
                               0,...
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