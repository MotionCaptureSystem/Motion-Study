function [X, Sig_X, varargout] = run_unscented_kf_recursive_condind(...
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
    options.groups = 1;
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
fprintf('Evaluating Timestep: 1')
for ii = 3:size(z,2) % for all timesteps
    % Line 2: Unscented transform on mu, Sig
    if ii ==3
        niter = 1;
    else
        niter = 1;
    end
    for tt = 1:length(num2str(ii-1))
        fprintf('\b');
    end
    fprintf(num2str(ii))
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
            [~, Rt] = g_handle(mu_bar, X(state_inds, ii-2), Rt_handle);
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
%         if isempty(links_prev)
%             Chi_bar = unscented_transform(mu_bar, param.lam, Sig_bar);
%         else
        full_mu = [X(state_inds_prev,ii);mu_bar];
        full_Sig = blkdiag(Sig_X(state_inds_prev,state_inds_prev,ii),Sig_bar);
        Chi_bar = unscented_transform(full_mu, param.lam, full_Sig);
%         end
%         figure (101)
%         hold on
%         if gg == 1
%             subplot(3,1,1);
%             plot(ii,norm(Sig_bar),'+b')
%         elseif gg==2
%             subplot(3,1,2);
%             plot(ii,norm(Sig_X(state_inds_prev,state_inds_prev,ii)),'or')
%             plot(ii,norm(Sig_bar),'+b')
%         else
%             subplot(3,1,3);
%             plot(ii,norm(Sig_X(state_inds_prev,state_inds_prev,ii)),'or')
%             plot(ii,norm(Sig_bar),'+b')
%         end
        % Detect & handle occlusions
        z_gg = zeros(2*npts_l*ncam,1);
        from_cam = zeros(2*npts_l*ncam,1);
        for cc = 1:ncam
            z_gg(2*npts_l*(cc-1)+(1:length([meas_inds_prev,meas_inds])),1) = z(nmeas*(cc-1)+[meas_inds_prev,meas_inds],ii);
            from_cam(2*npts_l*(cc-1)+(1:length([meas_inds_prev,meas_inds])),1) = cc*ones(length([meas_inds_prev,meas_inds]),1);
        end
        occlusion_ndx = find(isnan(z_gg)); % find ndx of occlusions
        z_minus_occlusions = z_gg; % create local msmt copy
        z_minus_occlusions(occlusion_ndx) = []; % strip occlusions out
        from_cam(occlusion_ndx) = [];
        % Line 7: Z_bar from msmt model on sig points
        %Z_bar = zeros(length(meas_inds), size(Chi_bar,2));
        
        Z_bar = zeros(size(z_gg,1),size(Chi_bar,2));
        for sigpt = 1:size(Chi_bar, 2)
            [Z_bar(:,sigpt), ~] = h_handle(Chi_bar(:,sigpt), ii);
        end
        Z_bar(occlusion_ndx,:) = []; % strip out occluded measurments
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
        
        %plot prediction and associated measurments
         for cc = 1:ncam
            figure (49+cc)
            hold on
            indx = from_cam == cc;
            z_hat_plot = reshape(z_hat(indx),2,[]);
            z_gg_plot = reshape(z_minus_occlusions(indx),2,[]);
            plot(z_hat_plot(1,:)',z_hat_plot(2,:)','+','Color','b')
            plot(z_gg_plot(1,:)',z_gg_plot(2,:)','o','Color','r')
%             text(z_gg_plot(1,:),z_gg_plot(2,:),num2str(gg))
%             text(z_hat_plot(1,:),z_hat_plot(2,:),num2str(gg))
            for bb = 1:size(z_hat_plot,2)
                plot([z_gg_plot(1,bb);z_hat_plot(1,bb)],[z_gg_plot(2,bb);z_hat_plot(2,bb)],'-k')

            end
%             if gg == 3
%             colors2 = hsv(size(Z_bar,2));
%             meas_c_plot = [];
%             for zz = 1:size(Z_bar,2)
%                 meas_cloud = h_handle(Chi_bar(:,zz),ii);
%                 meas_cloud(occlusion_ndx) = [];
%                 meas_c_plot((zz-1)*2+1:zz*2,:) = reshape(meas_cloud(indx),2,[]);
%             end
%             for point_num = 1:size(meas_c_plot,2)
%                 meas_plot_now = reshape(meas_c_plot(:,point_num),2,[]);
%                 plot([meas_plot_now(1,:),meas_plot_now(1,1)]',[meas_plot_now(2,:),meas_plot_now(2,1)]','-.','Color',colors2(point_num,:))
%             end
%             end
       end
        
         
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
%        figure (98)
%         if gg == 1
%             subplot(4,1,1); plot(ii,norm(K(1:3,:)),'+b'); hold on
%             subplot(4,1,2); plot(ii,norm(K(4:6,:)),'+r'); hold on
%         elseif gg == 2
%             subplot(4,1,1); plot(ii,norm(K(1:3,:)),'ob'); hold on
%             subplot(4,1,2); plot(ii,norm(K(4:6,:)),'or'); hold on
%             subplot(4,1,3); plot(ii,norm(K(7:10,:)),'or'); hold on
%         else
%             subplot(4,1,1); plot(ii,norm(K(1:3,:)),'*b'); hold on
%             subplot(4,1,2); plot(ii,norm(K(4:6,:)),'*r'); hold on
%             subplot(4,1,3); plot(ii,norm(K(7:10,:)),'*r'); hold on
%             subplot(4,1,4); plot(ii,norm(K(10:end,:)),'*r'); hold on
%         end
        % Line 12: Update state measurement and cov
        mu = full_mu + K*(z_minus_occlusions-z_hat); %use stripped msmt
        
        figure(100)
        hold on
        plot3(full_mu(1),full_mu(2),full_mu(3),'+b');
        plot3(mu(1),mu(2),mu(3),'or');
        plot3([full_mu(1);mu(1)],[full_mu(2);mu(2)],[full_mu(3);mu(3)],'-k');
%         delta_vec = z_minus_occlusions-z_hat;
        
%         for cc = 1:ncam
%             figure (101+cc)
%             hold on
%             indx = from_cam == cc;
%             delta_vec_cam = reshape(delta_vec(indx),2,[]);
%             n_meas_prev = length(meas_inds_prev)/2;
%             if n_meas_prev
%                 plot(delta_vec_cam(1,n_meas_prev)',delta_vec_cam(2,n_meas_prev)','*','Color','b')
%                 text(delta_vec_cam(1,n_meas_prev)',delta_vec_cam(2,n_meas_prev)',num2str(gg))
%             end
%             plot(delta_vec_cam(1,n_meas_prev+1:end)',delta_vec_cam(2,n_meas_prev+1:end)','^','Color','r')
%             text(delta_vec_cam(1,n_meas_prev+1:end)',delta_vec_cam(2,n_meas_prev+1:end)',num2str(gg))
%         end
%         if gg == 1
%             z_store{gg}(:,ii) = z_minus_occlusions-z_hat;
%         elseif gg == 2
%             z_store{gg}(:,ii) = z_minus_occlusions-z_hat;
%         else
%             z_store{gg}(:,ii) = z_minus_occlusions-z_hat;
%         end
%         updated_meas = h_handle(mu, ii);
%         updated_meas(occlusion_ndx) = [];
%         for cc = 10
%             figure (49+cc)
%             hold on
%             indx = from_cam == cc;
%             meas_plot = reshape(updated_meas(indx),2,[]);
%             z_gg_plot = reshape(z_minus_occlusions(indx),2,[]);
%             plot(meas_plot(1,:)',meas_plot(2,:)','*','Color','g')
%             text(meas_plot(1,:),meas_plot(2,:),num2str(gg))
%             for bb = 1:size(z_gg_plot,2)
%                 plot([z_gg_plot(1,bb);meas_plot(1,bb)],[z_gg_plot(2,bb);meas_plot(2,bb)],'-r')
%             end
%         end
%         %pause
%         if gg == 1
%             figure (99)
%             hold on
%             plot(ii,mu(1),'+r',ii,mu(2),'+b',ii,mu(3),'+g')
%             figure (100)
%             hold on
%             plot(ii,180/pi*mu(4),'+r',ii,180/pi*mu(5),'+b',ii,180/pi*mu(6),'+g')
%         elseif gg == 2
%             figure (99)
%             hold on
%             plot(ii,mu(1),'or',ii,mu(2),'ob',ii,mu(3),'og')
%             figure (100)
%             hold on
%             plot(ii,180/pi*mu(4),'or',ii,180/pi*mu(5),'ob',ii,180/pi*mu(6),'og')
%         else
%             figure (99)
%             hold on
%             plot(ii,mu(1),'*r',ii,mu(2),'*b',ii,mu(3),'*g')
%             figure (100)
%             hold on
%             plot(ii,180/pi*mu(4),'*r',ii,180/pi*mu(5),'*b',ii,180/pi*mu(6),'*g')
%         end
        
        Sig = full_Sig - K*S*K';
        
%         figure (97)
%         if gg == 1
%             subplot(3,1,1); plot(ii,norm(Sig(1:3,1:3)),'+b'); hold on
%             subplot(3,1,2); plot(ii,norm(Sig(4:6,4:6)),'+r'); hold on
%             subplot(3,1,2); plot(ii,norm(Sig(4:6,1:3)),'+c'); hold on
%             subplot(3,1,3); plot(ii,norm(Sig),'+g'); hold on
%         elseif gg == 2
%             subplot(3,1,1); plot(ii,norm(Sig(1:3,1:3)),'ob'); hold on
%             subplot(3,1,2); plot(ii,norm(Sig(4:6,4:6)),'or'); hold on
%             subplot(3,1,2); plot(ii,norm(Sig(4:6,1:3)),'oc'); hold on
%             subplot(3,1,3); plot(ii,norm(Sig),'og'); hold on
%         else
%             subplot(3,1,1); plot(ii,norm(Sig(1:3,1:3)),'*b'); hold on
%             subplot(3,1,2); plot(ii,norm(Sig(4:6,4:6)),'*r'); hold on
%             subplot(3,1,2); plot(ii,norm(Sig(4:6,1:3)),'*c'); hold on
%             subplot(3,1,3); plot(ii,norm(Sig),'*g'); hold on
%         end
        
        all_state_inds = [state_inds_prev,state_inds];
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
%                     if mu(xx) < mu_0(all_state_inds(xx)) - pi
%                         mu(xx) = mu(xx) + 2*pi;
%                     elseif mu(xx) > mu_0(all_state_inds(xx)) + pi 
%                         mu(xx) = mu(xx) - 2*pi;
%                     end
%                 end
%             end
%         end
                
        % Accumulate measurements into function outputs
        X([all_state_inds],ii) = mu;
        Sig_X(all_state_inds,all_state_inds,ii) = Sig;
        % norm_Q_log(ii) = max(max(Qt));

        outstruct.mu_bar([all_state_inds],ii) = mu;
        outstruct.Sig_bar([all_state_inds],[all_state_inds],ii) = Sig;
        end
    end
end
% norm_Q_log

% figure
% hold on
% if gg>=1
% plot([3:ii]',sqrt(sum(z_store{1}(1:2,3:end).*z_store{1}(1:2,3:end)))','+b')
% end
% if gg>=2
% plot([3:ii]',sqrt(sum(z_store{2}(1:2,3:end).*z_store{2}(1:2,3:end)))','ob',[3:ii]',sqrt(sum(z_store{2}(13:14,3:end).*z_store{2}(13:14,3:end)))','og')
% end
% if gg >=3
% plot([3:ii]',sqrt(sum(z_store{3}(1:2,3:end).*z_store{3}(1:2,3:end)))','*b',[3:ii]',sqrt(sum(z_store{3}(13:14,3:end).*z_store{3}(13:14,3:end)))','*g')
% end

if nargout > 2
    varargout{1} = outstruct;
end
    fprintf('\n')
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