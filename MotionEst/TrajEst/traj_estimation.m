function [eststruct, options] = traj_estimation(Cam, options)
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
%     [X_ukf, Sig_X_ukf]      = run_unscented_kf_recursive2(options.est.x_km1, options.est.Sigma_k, zeros(options.nstate,nsteps), ...
%                                                                    options.est.meas, options.est.state_update_model, options.est.msmt_model, ...
%                                                                    options.est.Rt_handle, options);
else
    [X_ukf, Sig_X_ukf, meas]   = run_unscented_kf_track2(options.est.x_km1, options.est.Sigma_k, zeros(options.nstate,nsteps), ...
                                                                   options.est.meas, options.est.state_update_model, options.est.msmt_model, ...
                                                                   options.est.Rt_handle, options, Cam);
    eststruct.n_correct = meas.n_correct;
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
                            options.link(ll).alphas,...
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