function Plot_Script_JS_diffdt(camstruct, eststruct, linetypes)%, options)
%% Plot 3D Points
options = eststruct(1).options;
camstruct = camstruct(options.est.cams);

nstruct = length(eststruct);
figure
hold on
%linetypes = {'o','s','^','+','.','-'};

figure
hold on
for ss = 1:nstruct
    
    options = eststruct(ss).options;
    %camstruct = camstruct(options.est.cams);
    features = eststruct(ss).ukf.Features;
    kinc = eststruct(ss).kinc;
    pts = options.plot.pts;
    npts = size([options.link.BFvecs],2);
    fs_c = options.fs_c;
    nsteps = size(features,2);
    cnt = 0;
    for pp = pts
        cnt = cnt+1;
%         if ss ~= 3
%             feat_manip = 1000*YPRTransform([0,-15/180*pi,-5/180*pi],[.300,1.200,.400])*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1]'*[features(3*(pp-1)+1:3*pp,:);ones(1,size(features,2))];
%         else 
        feat_manip = features(3*(cnt-1)+1:3*cnt,:);
%         end
        plot3(feat_manip(1,:)',feat_manip(2,:)', feat_manip(3,:)', linetypes{ss},'Color',options.plot.colors2(cnt,:))
    end
    xlabel('x (mm)',options.plot.fig_txt_props{:}); ylabel('y (mm)',options.plot.fig_txt_props{:}); zlabel('z (mm)',options.plot.fig_txt_props{:}); 
    axis tight
    axis equal
    h = gca;
    set(h,options.plot.fig_txt_props{:})
    textobj = findobj(h, 'type', 'text');
    lineobj = findobj(h, 'type', 'line');
    set(lineobj, 'LineWidth', 2);
    set(textobj,  options.plot.fig_txt_props{:});
end
plot_kin_chain(kinc, options, [1:5:length(kinc)]);

figure
for cc = 1:length(camstruct)
    if ~isempty(camstruct(cc).H)
        %H(:,:,cc) = 1000*YPRTransform([0,-15/180*pi,-5/180*pi],[.300,1.200,.400])*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1]'*camstruct(cc).H;
        H(:,:,cc) = camstruct(cc).H;
    end
end
CFPlot(H,50);
xlabel('x (mm)','FontSize', 18, 'FontName', 'Times New Roman'); 
ylabel('y (mm)','FontSize', 18, 'FontName', 'Times New Roman'); 
zlabel('z (mm)','FontSize', 18, 'FontName', 'Times New Roman');
axis equal
set(gca, options.plot.fig_txt_props{:});

% Define Intrinsic Parameters
% load int_cam301.mat

%% Plot Camera Space Measurments
%create measurements using pinhole camera model
% figure
% for ss = 1:nstruct
% options = EstStruct(ss).options;
% camstruct = camstruct(options.est.cams);
% features = EstStruct(ss).ukf.Features;
% points = zeros(2*length(options.est.cams)*npts,nsteps);
% for kk = 1:nsteps
%  [points(:,kk)] = CamNet(features(:,kk),camstruct);
% end
% 
% ii = 0;
% 
% for cc = 1:5:length(options.est.cams)
%     ii = ii+1;
%     for pp = 1:npts
%         subplot(2,4,ii);
%         hold on 
%         plot(points(2*npts*(cc-1)+2*(pp-1)+1,:)',points(2*npts*(cc-1)+2*(pp-1)+2,:)',options.plot.linespec1{pp})
%     end
%     title(sprintf('Cam %d',cc))
%     xlabel ('X (pixels)');
%     ylabel ('Y (pixels)');
% end
% end
%% Plot Joint Coords
%for ss = 1:nstruct
    options = eststruct(ss).options;
    link = options.link;
    %t = 1/fs_c*linspace(0,size(eststruct(ss).ukf.X,2)-1, size(eststruct(ss).ukf.X,2));
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
                for ss = 1:nstruct
                    t = 1/fs_c*linspace(0,size(eststruct(1).ukf.X,2)-1, size(eststruct(1).ukf.X,2));
                    t = [t(1),t(2:eststruct(ss).options.dt:end)];
                    legend_handles(length(legend_handles)+1) = plot(t,eststruct(ss).ukf.X(dof+dof_prev,:),linetypes{ss},'Color',options.plot.colors{dof});
                    hold on
                end
                clear title xlabel ylabel
                title([options.link_names{ll},' Displacements',],options.plot.fig_txt_props{:})
                xlabel('time (s)',options.plot.fig_txt_props{:})
                ylabel('Displacement',options.plot.fig_txt_props{:})
                axis tight
            else
                clear title xlabel ylabel
                for ss = 1:nstruct
                    t = 1/fs_c*linspace(0,size(eststruct(1).ukf.X,2)-1, size(eststruct(1).ukf.X,2));
                    t = [t(1),t(2:eststruct(ss).options.dt:end)];
                    legend_handles(length(legend_handles)+1) = plot(t,180/pi*eststruct(ss).ukf.X(dof+dof_prev,:),linetypes{ss},'Color',options.plot.colors{dof});
                    hold on
                end
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
%end
