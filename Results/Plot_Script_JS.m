function Plot_Script_JS(camstruct, eststruct, options)
%% Plot 3D Points
camstruct = camstruct(options.est.cams);
features = eststruct.ukf.Features;

kinc = eststruct.kinc;
pts = options.plot.pts;
npts = size([options.link.BFvecs],2);
fs_c = options.fs_c;
nsteps = size(features,2);
figure
hold on
cnt = 0;
for pp = 1:length(pts(2:end))
    cnt = cnt+1;
    feat_manip = eye(4)*[features(3*(pp-1)+1:3*pp,:);ones(1,size(features,2))];%1000*YPRTransform([0,-15/180*pi,-5/180*pi],[.300,1.200,.400])*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1]';
    plot3(feat_manip(1,:)',feat_manip(2,:)', feat_manip(3,:)', '-.','Color',options.plot.colors(cnt,:))
end

plot_kin_chain(kinc, options, [1:5:length(kinc)]);
xlabel('x (mm)',options.plot.fig_txt_props{:}); ylabel('y (mm)',options.plot.fig_txt_props{:}); zlabel('z (mm)',options.plot.fig_txt_props{:}); 
H = reshape([camstruct.H],4,4,[]);
CFPlot(H,20)

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
points = zeros(2*length(options.est.cams)*npts,nsteps);
for kk = 1:nsteps
 [points(:,kk)] = CamNet(features(:,kk),camstruct);
end

ii = 0;
figure
for cc = 1:5:length(options.est.cams)
    ii = ii+1;
    for pp = 1:npts
        subplot(2,4,ii);
        hold on 
        plot(points(2*npts*(cc-1)+2*(pp-1)+1,:)',points(2*npts*(cc-1)+2*(pp-1)+2,:)','color',options.plot.colors(pp,:))
    end
    title(sprintf('Cam %d',cc))
    xlabel ('X (pixels)');
    ylabel ('Y (pixels)');
end

%% Plot Joint Coords
link = options.link;
t = 1/fs_c*linspace(0,size(eststruct.ukf.X,2)-1, size(eststruct.ukf.X,2));
dof_prev = 0;
dof_int = [];
for ll = get_group_links(link,options.groups)
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
            hold on
            legend_handles(length(legend_handles)+1) = plot(t,eststruct.ukf.X(dof+dof_prev,:),options.plot.linespec1{dof});
            clear title xlabel ylabel
            title([options.link(ll).nnames,' Displacements',],options.plot.fig_txt_props{:})
            xlabel('time (s)',options.plot.fig_txt_props{:})
            ylabel('Displacement',options.plot.fig_txt_props{:})
            axis tight
        else
            clear title xlabel ylabel
            hold on
            legend_handles(length(legend_handles)+1) = plot(t,180/pi*eststruct.ukf.X(dof+dof_prev,:),options.plot.linespec1{dof});
            title([options.link(ll).nnames,' Rotations'],options.plot.fig_txt_props{:})
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

