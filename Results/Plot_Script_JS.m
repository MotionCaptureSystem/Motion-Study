%% Plot 3D Points
clear title xlabel ylabel
q = EstPoints.filt.ukf.X;
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
            kinc(kk).link(ll).H=DHTransforms(q_lk+options.link(ll).thetas,...
                        options.link(ll).alphas,...
                        options.link(ll).disps,...
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
    for kk = 1:nsteps
    X = [eye(3,3),zeros(3,1)]*hnode2node(kinc(kk),options,1,ll)*[options.link(ll).BFvecs;ones(1,n_bf_pts)];
    features(3*(n_pts_tot-n_bf_pts)+1:3*n_pts_tot,kk) = X(:);
    end
    assoc(:,(n_pts_tot-n_bf_pts)+1:n_pts_tot) = [ll*ones(1,n_bf_pts);1:n_bf_pts];
end

EstPoints.filt.ukf.Features = features;

figure
hold on
cnt = 0;
for pp = [1,2,3,6,7,8,11,12,14,15,17]
    cnt = cnt+1;
    feat_manip = 1000*YPRTransform([0,-15/180*pi,-5/180*pi],[.300,1.200,.400])*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1]'*[features(3*(pp-1)+1:3*pp,:);ones(1,size(features,2))];
    plot3(feat_manip(1,:)',feat_manip(2,:)', feat_manip(3,:)', '-.','Color',options.plot.colors2(cnt,:))
end

plot_kin_chain(kinc, options, [1:5:length(kinc)]);
xlabel('x (mm)',options.plot.fig_txt_props{:}); ylabel('y (mm)',options.plot.fig_txt_props{:}); zlabel('z (mm)',options.plot.fig_txt_props{:}); 
axis tight
axis equal
h = gca;
set(h,options.plot.fig_txt_props{:})
textobj = findobj(h, 'type', 'text');
lineobj = findobj(h, 'type', 'line');
set(lineobj, 'LineWidth', 2);
set(textobj,  options.plot.fig_txt_props{:});

% Define Intrinsic Parameters
% load int_cam301.mat


%% Plot Camera Space Measurments
%create measurements using pinhole camera model
points = zeros(2*length(options.cams)*npts,nsteps);
for kk = 1:nsteps
 [points(:,kk)] = CamNet(features(:,kk),Cam);
end

ii = 0;
figure
for cc = 1:5:length(options.cams)
    ii = ii+1;
    for pp = 1:npts
        subplot(2,4,ii);
        hold on 
        plot(points(2*n_pts_tot*(cc-1)+2*(pp-1)+1,:)',points(2*n_pts_tot*(cc-1)+2*(pp-1)+2,:)',options.plot.linespec1{pp})
    end
    title(sprintf('Cam %d',cc))
    xlabel = ('X (pixels)');
    ylabel = ('Y (pixels)');
end

%% Plot Joint Coords
link = options.link;
t = 1/120*linspace(0,size(EstPoints.filt.ukf.X,2)-1, size(EstPoints.filt.ukf.X,2));
dof_prev = 0;
dof_int = [];
for ll = get_group_links(link,options.groups);
nDof = link(ll).nDof;
tDof = link(ll).tDof;
legend_handles = [];
    for dof = 1:nDof
        if dof>1
            if tDof(dof) ~= tDof(dof-1)
                h = gca;
                set(h,options.plot.fig_txt_props{:})
                textobj = findobj(h, 'type', 'text');
                lineobj = findobj(h, 'type', 'line');
                set(lineobj, 'LineWidth', 2);
                set(textobj,  options.plot.fig_txt_props{:});
                handle = legend(legend_handles, options.dof_names{dof_prev+1:dof_prev+nDof}, 'Location','NorthEastOutside');
                set(handle,  options.plot.fig_txt_props{:});
                legend_handles = [];
                dof_int = dof_prev+dof-1;
                fig_id = figure;
            end
        else
            fig_id = figure;
        end
        if tDof(dof)
            legend_handles(length(legend_handles)+1) = plot(t,EstPoints.filt.ukf.X(dof+dof_prev,:),options.plot.linespec1{dof});
            hold on
            clear title xlabel ylabel
            title([options.link_names{ll},' Displacements',],options.plot.fig_txt_props{:})
            xlabel('time (s)',options.plot.fig_txt_props{:})
            ylabel('Displacement',options.plot.fig_txt_props{:})
            axis tight
        else
            clear title xlabel ylabel
            legend_handles(length(legend_handles)+1) = plot(t,180/pi*EstPoints.filt.ukf.X(dof+dof_prev,:),options.plot.linespec1{dof});
            hold on
            title([options.link_names{ll},' Rotations'],options.plot.fig_txt_props{:})
            xlabel('time (s)',options.plot.fig_txt_props{:})
            ylabel('Angle (deg)',options.plot.fig_txt_props{:})
            axis tight
        end
    end
    h = gca;
    set(h,options.plot.fig_txt_props{:})
    textobj = findobj(h, 'type', 'text');
    lineobj = findobj(h, 'type', 'line');
    set(lineobj, 'LineWidth', 2);
    set(textobj,  options.plot.fig_txt_props{:});
    if ~isempty(dof_int)
        handle = legend(legend_handles, options.dof_names{dof_int+1:dof_prev+nDof}, 'Location','NorthEastOutside');
        set(handle,  options.plot.fig_txt_props{:});
        dof_int = [];
    else
         handle = legend(legend_handles, options.dof_names{dof_prev+1:dof_prev+nDof}, 'Location','NorthEastOutside');
        set(handle,  options.plot.fig_txt_props{:});
    end
    
    dof_prev = dof_prev+nDof;
    %fig_id = gcf;
    %Do the figures need to be saved?
    if options.plot.savefig
        print('-r600', '-dpdf', strcat(options.plot.savepath,filesep,'JointCoord_',options.link_names{ll},'.pdf'))
        %imwrite(fig_id,strcat(options.plot.savepath,filesep,'JointCoord_',options.link_names{ll},'.png'),'png')
    end
end

%% Plot the Joint Space Covariances

% link = options.link;
% t = 1/120*linspace(0,size(EstPoints.filt.ukf.X,2)-1, size(EstPoints.filt.ukf.X,2));
% dof_prev = 0;
% for ll = get_group_links(link,options.groups);
% nDof = link(ll).nDof;
% tDof = link(ll).tDof;
%     for dof = 1:nDof
%         if dof>1
%             if tDof(dof) ~= tDof(dof-1)
%                 figure
%             end
%         else
%             figure
%         end
%         if tDof(dof)
%             plot(t,EstPoints.filt.ukf.Sig_X(dof+dof_prev,:),options.plot.linespec1{dof})
%             hold on
%             clear title xlabel ylabel
%             title(['Covariance of Displacements of ',options.link_names{ll}])
%             xlabel('time (s)')
%             ylabel('Displacement (mm)')
%         else
%             clear title xlabel ylabel
%             plot(t,180/pi*EstPoints.filt.ukf.Sig_X(dof+dof_prev,:),options.plot.linespec1{dof})
%             hold on
%             title(['Covariance of YPR Rotations of ',options.link_names{ll}])
%             xlabel('time (s)')
%             ylabel('Angle (deg)')
%         end
%     end
%     dof_prev = dof_prev+nDof;
% end
