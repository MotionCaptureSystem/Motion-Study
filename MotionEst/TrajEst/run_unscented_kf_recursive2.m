function [X, Sig_X, varargout] = run_unscented_kf_recursive2(...
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

if nargin < 9
    param = struct();
else
    param = varargin{1};
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

% figure
% hold on
% for yy = 1:size(z,1)/2
%     plot(z((yy-1)*2+1,3:end)',z(2*yy,3:end)')
% end

% Sig = Sig_0;
% norm_Q_log = zeros(size(u,2),1);

for ii = 3:size(z,2) % for all timesteps
    % Line 2: Unscented transform on mu, Sig
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
                npts_l = size([link(links).BFvecs],2);
                n   = sum([link(links).nDof]);
                state_inds = [link(links).StateInds];
                meas_inds = [link(links).MeasInds];
                links_prev = get_group_links(link,1:gg-1);
                state_inds_prev = [link(links_prev).StateInds];
            elseif strcmp(options.est.type, 'point')
                npts_l = length(options.pts);
                n = 3*npts_l;
                state_inds = 1:n;
                meas_inds = 1:2*npts_l;
                state_inds_prev = [];
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
                [Chi_star(:,sigpt), ~] = g_handle(Chi_prev(:,sigpt), Rt_handle);
            elseif strcmp(options.est.type, 'joint')
                [Chi_star(:,sigpt), ~] = g_handle(Chi_prev(:,sigpt), X(state_inds,ii-2), links, Rt_handle);
            end
        end

        % Line 4&5: Form mu_bar and Sig_bar from weighted sum of Chi_star
        mu_bar = Chi_star * wm';
        if strcmp(options.est.type, 'point')
            [~, Rt] = g_handle(mu_bar, Rt_handle);
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
        Chi_bar = unscented_transform(mu_bar, param.lam, Sig_bar);

        % Detect & handle occlusions
        z_gg = zeros(2*npts_l*ncam,1);
        for cc = 1:ncam
            z_gg(2*npts_l*(cc-1)+(1:length(meas_inds)),1) = z(nmeas*(cc-1)+meas_inds,ii);
        end
        occlusion_ndx = find(isnan(z_gg)); % find ndx of occlusions
        z_minus_occlusions = z_gg; % create local msmt copy
        z_minus_occlusions(occlusion_ndx) = []; % strip occlusions out

        % Line 7: Z_bar from msmt model on sig points
        %Z_bar = zeros(length(meas_inds), size(Chi_bar,2));
        
        Z_bar = zeros(size(z_gg,1),size(Chi_bar,2));
        for sigpt = 1:size(Chi_bar, 2)
            [Z_bar(:,sigpt), ~] = h_handle([X(state_inds_prev,ii);Chi_bar(:,sigpt)], ii);
        end
        Z_bar(occlusion_ndx,:) = []; % strip out occluded measurments

        % Line 8--10: z_hat (mean msmt) and S (msmt cov) from weighted sum
        % of Z_bar, Sig_xz from "del terms"
        z_hat = Z_bar * wm';
        [~, Qt] = h_handle([X(state_inds_prev,ii);mu_bar], ii);
        del_Z = (Z_bar - repmat(z_hat,1,size(Z_bar,2)));
        del_Sig = Chi_bar - repmat(mu_bar,1,size(Chi_bar,2));
        S = zeros(length(z_hat), length(z_hat));
        Sig_xz = zeros(length(mu_bar), length(z_hat));
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
        mu = mu_bar + K*(z_minus_occlusions-z_hat); %use stripped msmt
%         if gg == 3
%             figure
%             hold on
%             plot(z_minus_occlusions(1:2:end),z_minus_occlusions(2:2:end),'+k')
%             plot(z_hat(1:2:end),z_hat(2:2:end),'+m')
%         end
        Sig = Sig_bar - K*S*K';                     
%         if strcmp(options.est.type, 'joint')
%             if gg == 1
%                 for xx = 4:length(mu)
%                     if mu(xx) < mu_0(xx) - pi
%                         mu(xx) = mu(xx) + 2*pi;
%                     elseif mu(xx) > mu_0(xx) + pi 
%                         mu(xx) = mu(xx) - 2*pi;
%                     end
%                 end
%             else
%                 for xx = 1:length(mu)
%                     if mu(xx) < mu_0(state_inds(xx)) - pi
%                         mu(xx) = mu(xx) + 2*pi;
%                     elseif mu(xx) > mu_0(state_inds(xx)) + pi 
%                         mu(xx) = mu(xx) - 2*pi;
%                     end
%                 end
%             end
%         end
                
        % Accumulate measurements into function outputs
        X(state_inds,ii) = mu;
        Sig_X(state_inds,state_inds,ii) = Sig;
        % norm_Q_log(ii) = max(max(Qt));

        outstruct.mu_bar(state_inds,ii) = mu_bar;
        outstruct.Sig_bar(state_inds,state_inds,ii) = Sig_bar;
        end
    end
end
% norm_Q_log
if nargout > 2
    varargout{1} = outstruct;
end
    
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