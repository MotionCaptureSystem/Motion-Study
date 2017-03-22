function [X, Sig_X, varargout] = run_unscented_kf_track2(...
    mu_0, Sig_0, u, z, g_handle, h_handle, Rt_handle, options, varargin)
% RUN_UNSCENTED_KF
%  [X, Sig_X] = RUN_UNSCENTED_KF(mu_0, Sig_0, u, z, g_handle, h_handle)
%    runs unscented kalman filter on input datasets u and z (described
%    below) with input motion and sensor models (also described below)
%    
%    INPUTS:
%    mu_0 = Initial state estimate vector (size nx1)
%    Sig_0 = Initial covariance estimate (square pos-def matrix size nxn)
%    u = Input trajectory matrix (size mxT, T = num-of-timesteps)
%    z = Msmt matrix (size pxT, T = num-of-timesteps)
%    g_handle = func handle to Motion model, see below for form
%    h_handle = func handle to Sensor model, see below for form
%
%    OUTPUTS:
%    X = State estimate matrix (size nxT) for all time T
%    Sig_X = State covariance (size nxnxT) for all time T
%
%    FUNCTION HANDLES:
%    [xk, Rk] = g_handle(x,u,t) returns the next state prediction (xk) from
%      some prediction model on the current state x, the input u, and
%      (optionally) the timestep t.  The dimensions of xk must be equal to
%      the dimensions of x (also equivalent to dim mu_0 above).  Also
%      returns the associated covariance Rk.
%    [zk, Qk] = h_handle(x,t) returns the measurement prediction (zk) from
%      some measurement model on the current state x and (optionally) the
%      timestep t.  Also returns the associated covariance Qk.
% 
%  [..., outstruct] = RUN_UNSCENTED_KF(..., param) allows the input of
%    an optional param struct to control the algorithm and the output of
%    optional params the user may want to investigate performance
%
%    param.lam = parameter controling spread of sigma pts.  lam=1 by
%     default, (lam+n) = 3 good for gaussians (according to julier1997)
%
%    outstruct.mu_bar = [nxT] matrix of motion-model state estimates
%    outstruct.Sig_bar = [nxnxT] matrix of motion-model state covariances 

if strcmp(options.est.type,'joint')
    link= options.link;
    links = get_group_links(link,options.groups);
    nstate = sum([link(links).nDof]);
    nmeas = 2*size([link(links).BFvecs],2);
    ncam = size(z,1)/nmeas;
elseif strcmp(options.est.type,'point')
    nstate = 3*length(options.pts);
    ncam   = length(options.cams);
    nmeas  = 2*length(options.pts);
end

if nargin >= 9
    camstruct = varargin{1};
end

if nargin < 10
    param = struct();
else
    param = varargin{2};
end

% Note: code follows variable naming in thrun2005probab_robot textbook

% Specify default parameters
default.lam = 2; % Controls spread of sigma points

default.beta = 2; % Advanced param, optimal = 2 for gaussians
default.alpha = .25; %1; % Advanced param, value guessed by hgm

% Use passed-in params, or defaults if blank
param = populate_struct_with_defaults(param, default);

% Initialize outputs
X = zeros(nstate,size(z,2)+2);
X(:,1) = mu_0; 
X(:,2) = mu_0;
z      = [zeros(size(z,1),2),z];
Sig_X = zeros(nstate,nstate,size(z,2));
Sig_X(:,:,1) = Sig_0;
Sig_X(:,:,2) = Sig_0;

t_world = [options.tstart-2,options.tstart-1,options.tstart:options.tstop];
% figure
% hold on
% for yy = 1:size(z,1)/2
%     plot(z((yy-1)*2+1,3:end)',z(2*yy,3:end)')
% end

% Sig = Sig_0;
% norm_Q_log = zeros(size(u,2),1);
fprintf('Estimating Time Step: ');
z_gg_auto_all{1} = NaN*zeros(126,size(z,2));
z_gg_auto_all{2} = NaN*zeros(294,size(z,2));
z_gg_auto_all{3} = NaN*zeros(size(z));
for ii = 3:size(z,2) % for all timesteps
    % Line 2: Unscented transform on mu, Sig
    fprintf('%i',ii);
    if ii ==3
        niter = 1;
    else
        niter = 1;
    end
    
    for gg = options.groups
        for ss = 1:niter
            %added this if construct to determine what type of estimation
            %is done.  If joint estimation, then link attributes are
            %needed, otherwise all points are estimated with no
            %connectivity assumed.
            if strcmp(options.est.type, 'joint')
                links = get_group_links(link,gg);
                n   = sum([link(links).nDof]);
                state_inds = [link(links).StateInds];
                meas_inds = [link(links).MeasInds];
                links_prev = get_group_links(link,1:gg-1);
                state_inds_prev = [link(links_prev).StateInds];
                meas_inds_prev = [link(links_prev).MeasInds];
                npts_l = size([link([links_prev,links]).BFvecs],2);
            elseif strcmp(options.est.type, 'point')
                npts_l = length(options.pts);
                n = 3*npts_l;
                state_inds = 1:n;
                meas_inds = 1:2*npts_l;
                state_inds_prev = [];
                meas_inds_prev = [];
            end
        if ss>1
            mu  = X(state_inds,ii);
        else
            mu  = X(state_inds,ii-1);
        end
        
        Sig = Sig_X(state_inds,state_inds,ii-1);
        
        % build w col-vectors (weights of sigma points)
        wm = [];
        wc = [];
        wm(1) = param.lam/(n + param.lam);
        wc(1) = wm(1) + (1-param.alpha^2 + param.beta);
        wm(2:2*n+1) = 1/(2*(n+param.lam))*ones(2*n,1);
        wc(2:2*n+1) = wm(2:2*n+1);
        %for zz = 1:niter
        Chi_prev = unscented_transform(mu, param.lam, Sig);

        % Line 3: Form Chi_star via motion model on sig points
        Chi_star = zeros(size(Chi_prev));
        for sigpt = 1:size(Chi_star, 2)
            if strcmp(options.est.type, 'point')
                [Chi_star(:,sigpt), ~] = g_handle(Chi_prev(:,sigpt), X(state_inds,ii-2), Rt_handle);
            elseif strcmp(options.est.type, 'joint')
                [Chi_star(:,sigpt), ~] = g_handle(Chi_prev(:,sigpt), X(state_inds,ii-2), links, Rt_handle);
            end
        end

        % Line 4&5: Form mu_bar and Sig_bar from weighted sum of Chi_star
        mu_bar = Chi_star * wm';
        if strcmp(options.est.type, 'point')
            [~, Rt] = g_handle(mu_bar, X(state_inds,ii-2), Rt_handle);
        elseif strcmp(options.est.type, 'joint')
            [~, Rt] = g_handle(mu_bar, X(state_inds, ii-2), links, Rt_handle);
        end
        del_Sig = (Chi_star - repmat(mu_bar,1,size(Chi_star,2)));
        Sig_bar = zeros(size(Sig));
        for ndx = 1:size(del_Sig,2)
            Sig_bar = Sig_bar + wc(ndx)*del_Sig(:,ndx)*del_Sig(:,ndx)';
        end
        Sig_bar = Sig_bar + Rt;

        % Line 6: Chi_bar from Unscented transform
        full_mu = [X(state_inds_prev,ii);mu_bar];
        full_Sig = blkdiag(Sig_X(state_inds_prev,state_inds_prev,ii),Sig_bar);
        Chi_bar = unscented_transform(full_mu, param.lam, full_Sig);

        % Line 7: Z_bar from msmt model on sig points
        %Z_bar = zeros(length(meas_inds), size(Chi_bar,2));
        Z_bar = zeros(ncam*length([meas_inds_prev,meas_inds]),size(Chi_bar,2));
        for sigpt = 1:size(Chi_bar, 2)
            [Z_bar(:,sigpt), ~] = h_handle(Chi_bar(:,sigpt), ii);
        end
        
%         figure
%         im = imread([options.path,filesep,'Cam302',filesep,num2str(t_world(ii)),'.png']);
%         imshow(im)
%         hold on
%         plot(Z_bar(26+1:2:26+length([meas_inds_prev,meas_inds]),1), Z_bar(26+2:2:26+length([meas_inds_prev,meas_inds]),1), '+c')
%         for pt = 1:length([meas_inds_prev,meas_inds])/2
%             plot([Z_bar(26+2*(pt-1)+1,2:end),Z_bar(26+1,2)]', [Z_bar(26+2*pt,2:end),Z_bar(26+2*pt,2)]', '*-m')
%         end
        % Redefine weights
        n  = length(full_mu);
        wm = [];
        wc = [];
        wm(1) = param.lam/(n + param.lam);
        wc(1) = wm(1) + (1-param.alpha^2 + param.beta);
        wm(2:2*n+1) = 1/(2*(n+param.lam))*ones(2*n,1);
        wc(2:2*n+1) = wm(2:2*n+1);
        % Line 8--10: z_hat (mean msmt) and S (msmt cov) from weighted sum
        % of Z_bar, Sig_xz from "del terms"
        z_hat = Z_bar * wm';
        z_hat_km1{gg}(:,ii) = z_hat;
        
        z_hat_camcol = reshape(z_hat,2,[],ncam);
%         plot(z_hat_camcol(26+1:2:26+length([meas_inds_prev,meas_inds])),z_hat_camcol(26+2:2:26+length([meas_inds_prev,meas_inds])),'xg')
%         pause
        % Detect & handle occlusions
        z_gg = zeros(2*npts_l*ncam,1);
        from_cam = zeros(2*npts_l*ncam,1);
        for cc = 1:ncam
            %Determine the time index for this camera
            t_cam = t_world(ii)+floor(camstruct(cc).sync_del*119.88)-camstruct(cc).start_frame+1;
            %Get the ground truth correspondences
            z_gg(2*npts_l*(cc-1)+(1:length([meas_inds_prev,meas_inds])),1) = z(nmeas*(cc-1)+[meas_inds_prev,meas_inds],ii);
            
            %Read the image from this timestep for which correspondence
            %will be computed
            im_k   = imread([options.path,filesep,'Cam',num2str(options.est.cams(cc)),filesep,num2str(t_world(ii)),'.png']);
            if size(im_k,3)>1
            im_k   = rgb2gray(im_k);
            end
            %read the image from the previous timestep
            im_km1 = imread([options.path,filesep,'Cam',num2str(options.est.cams(cc)),filesep,num2str(t_world(ii)-1),'.png']);
            if size(im_km1,3)>1
            im_km1 = rgb2gray(im_km1);
            end
            
            %Get the point numbers to correspond
            if strcmp(options.est.type,'joint') %joint estimation point numbers are based on the pt association
                pt_nums = [camstruct(cc).pt_assoc{[links_prev,links]}];
            else %point estimation point numbers are declared in options. 
                pt_nums = options.pts;
            end
            
            phi_hat = CameraDistortion(reshape(z_hat_camcol(:,:,cc),[],1),camstruct(cc));   %estimated point locations
            phi     = reshape(camstruct(cc).pts(:,t_cam,pt_nums),[],1); %uncorresponded features
            phi_km1 = reshape(camstruct(cc).pts(:,t_cam-1,pt_nums),[],1); %previous corresponded features 
            %Run the optical flow correspondence
            phi_corr = corresp_optflow(phi_hat,phi,phi_km1,im_km1,im_k);
            %Create a figure to display the correspondences
            z_gg_auto_all{gg}(2*length([camstruct(cc).pt_assoc{[links_prev,links]}])*(cc-1)+1:2*length([camstruct(cc).pt_assoc{[links_prev,links]}])*cc,ii) = phi_corr;
            if gg == options.groups(end)
                n_correct(cc,ii) = sum(phi_corr==phi)/2;
            end
            if 0%any(~isnan(phi)) && gg == 3
                figure
                imshow(im_k)
                hold on
                plot(phi_km1(1:2:end),phi_km1(2:2:end),'og') 
                plot(phi_hat(1:2:end),phi_hat(2:2:end),'+c')
                plot(phi(1:2:end),phi(2:2:end),'.m')

                for pp = 1:length(phi_hat)/2
                   plot([phi_hat(2*(pp-1)+1);phi_corr(2*(pp-1)+1)], [phi_hat(2*pp);phi_corr(2*pp)],'-y')
                end
                title(sprintf('Cam: %i Timestep: %i',cc,t_world(ii)))
            end
        end
                       
         occlusion_ndx = find(isnan(z_gg_auto_all{gg}(:,ii))); % find ndx of occlusions
         z_minus_occlusions = z_gg_auto_all{gg}(:,ii); % create local msmt copy
        %occlusion_ndx = find(isnan(z_gg)); % find ndx of occlusions
        %z_minus_occlusions = z_gg; % create local msmt copy
        z_minus_occlusions(occlusion_ndx) = []; % strip occlusions out
        %from_cam(occlusion_ndx) = [];
        Z_bar(occlusion_ndx,:) = []; % strip out occluded measurements
        z_hat(occlusion_ndx,:) = [];

        [~, Qt] = h_handle(full_mu, ii);
        %Qt = Qt((end-length(meas_inds)+1):end,(end-length(meas_inds)+1):end);
        del_Z = (Z_bar - repmat(z_hat,1,size(Z_bar,2)));
        del_Sig = Chi_bar - repmat(full_mu,1,size(Chi_bar,2));
        S = zeros(length(z_hat), length(z_hat));
        Sig_xz = zeros(length(full_mu), length(z_hat));
        for ndx = 1:size(del_Z,2)
            S = S + wc(ndx)*del_Z(:,ndx)*del_Z(:,ndx)';
            Sig_xz = Sig_xz + wc(ndx)*del_Sig(:,ndx)*del_Z(:,ndx)';
        end
        
        Qt(occlusion_ndx,:) = [];  % strip occlusion rows out
        Qt(:, occlusion_ndx) = []; % strip occlusion cols out
        S = S + Qt;

        % Line 11: K (kalman msmt gain)
        K = ((S')\Sig_xz')'; % just avoid inv(S)

        % Line 12: Update state measurement and cov
        mu = full_mu + K*(z_minus_occlusions-z_hat); %use stripped msmt
        
        Sig = full_Sig - K*S*K';
        
        all_state_inds = [state_inds_prev,state_inds];
                
        % Accumulate measurements into function outputs
        X([all_state_inds],ii) = mu;
        Sig_X(all_state_inds,all_state_inds,ii) = Sig;

        % norm_Q_log(ii) = max(max(Qt));

        outstruct.mu_bar([all_state_inds],ii) = mu;
        outstruct.Sig_bar([all_state_inds],[all_state_inds],ii) = Sig;
        
        end
    end
    
    if ii<size(z,2)
        cnt = 0;
        while cnt<length(num2str(ii))
            cnt = cnt+1;
            fprintf('\b')
        end
    end
end

outstruct.z_auto_corr = z_gg_auto_all;
outstruct.n_correct = n_correct;

if nargout > 2
    varargout{1} = outstruct;
end
% figure 
% hold on
% colors = hsv(ncam);
% 
% for cc = 1:ncam
%     plot([1:size(n_correct,2)]',n_correct(cc,:)','color',colors(cc,:))
% end

end %function

function Chi = unscented_transform(mu, lam, Sig)
Chi = zeros(length(mu), 2*length(mu)+1);
Chi(:,1) = mu; % sigpt_0 is Chi_prev(1)
sqrtSig = sqrtm(Sig); % Matrix square-root.  Possibly could do faster
                      % with Cholesky decomp? (for future ref)
for ndx = 1:length(mu)
    sigpt_adj = sqrt(length(mu)+lam)*sqrtSig(:,ndx);
    Chi(:,ndx+1) =            mu + sigpt_adj;
    Chi(:,ndx+1+length(mu)) = mu - sigpt_adj;
end

end %function