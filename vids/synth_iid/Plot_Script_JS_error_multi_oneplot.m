function Plot_Script_JS_error_multi_oneplot(camstruct, eststruct,linetypes, options)
%% Plot 3D Points

camstruct = camstruct(options.est.cams);

nstruct = length(eststruct);
figure
hold on
%linetypes = {'--','.-','-','-+'};

figure
hold on
pts = options.plot.pts;
colors = hsv(length(pts));
for ss = 1:nstruct
    
    %camstruct = camstruct(options.est.cams);
    features = eststruct(ss).ukf.Features;
    kinc = eststruct(ss).kinc;
    
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
        plot3(feat_manip(1,:)',feat_manip(2,:)', feat_manip(3,:)', linetypes{ss},'Color',colors(cnt,:))
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
options.dof_names(1) = {'Z'};
options.dof_names(3) = {'X'};
link = options.link;
t = 1/fs_c*linspace(0,size(eststruct(ss).ukf.X,2)-1, size(eststruct(ss).ukf.X,2));
dof_prev = 0;
dof_int = [];
nrows = ceil(size(eststruct(ss).ukf.X,1)/4);
ndof_plot_group = [sum([options.link(1:3).nDof]),sum([options.link(4:7).nDof]),sum([options.link(8:13).nDof]),...
                    sum([options.link(14:19).nDof]),sum([options.link(20:end).nDof])];
group_last = 0;
for ll = get_group_links(link,options.groups)
    nDof = link(ll).nDof;
    tDof = link(ll).tDof;
    legend_handles = [];
    if group_last ~= link(ll).Group
        figure
        group_last = link(ll).Group;
        group_dof = 0;
    end

    for dof = 1:nDof

        subplotfill(ceil(ndof_plot_group(link(ll).Group)/3),3,group_dof+dof)
        if tDof(dof)

            for ss = 2:nstruct
                legend_handles(length(legend_handles)+1) = plot(t,eststruct(ss).ukf.X(dof+dof_prev,:)-eststruct(1).ukf.X(dof+dof_prev,:),linetypes{ss},'Color','k');
                hold on
            end
            axis tight
            grid on

            ylabel([options.dof_names{dof_prev + dof},' (mm)'],'FontWeight','Bold','FontName','Sans Serif','fontsize',16)%,options.plot.fig_txt_props{:})
            h = gca;
            set(h,'FontWeight','Bold','FontName', 'Sans Serif','fontsize',16)
            textobj = findobj(h, 'type', 'text');
            lineobj = findobj(h, 'type', 'line');
            set(lineobj, 'LineWidth', 1.2);
            set(textobj,  'FontWeight','Bold','FontName', 'Sans Serif','fontsize',16);
        else
            clear title xlabel ylabel
            for ss = 2:nstruct
                legend_handles(length(legend_handles)+1) = plot(t,180/pi*(eststruct(ss).ukf.X(dof+dof_prev,:)-eststruct(1).ukf.X(dof+dof_prev,:)),linetypes{ss},'Color','k');
                hold on
            end
            grid on
            axis tight
%                 if dof == 1
%                     title([options.dof_names{dof_prev+dof},' Rotations'],options.plot.fig_txt_props{:})
%                 end
             if (dof + group_dof >= ndof_plot_group(link(ll).Group)-2)
                 xlabel('time (s)','FontWeight','Bold','FontName', 'Sans Serif','fontsize',16)
             else
                 set(gca,'XTickLabelMode','Manual');
                 set(gca,'XTickLabel',[]);
             end
             ylabel([options.dof_names{dof_prev+dof},' (deg)'],'FontWeight','Bold','FontName', 'Sans Serif','fontsize',16)%,options.plot.fig_txt_props{:})
%                 axis tight
%                 h = gca;
            h = gca;
            set(h,'FontWeight','Bold','FontName', 'Sans Serif','fontsize',16)
            textobj = findobj(h, 'type', 'text');
            lineobj = findobj(h, 'type', 'line');
            set(lineobj, 'LineWidth', 1.2);
            set(textobj,  'FontWeight','Bold','FontName', 'Sans Serif','fontsize',16);
        end
    end

    dof_prev = dof_prev+nDof;
    group_dof = group_dof+nDof;
     %Do the figures need to be saved?
    if options.plot.savefig
        print('-r600', '-dpdf', strcat(options.plot.savepath,filesep,'JointCoord_',options.link_names{ll},'.pdf'))
        %imwrite(fig_id,strcat(options.plot.savepath,filesep,'JointCoord_',options.link_names{ll},'.png'),'png')
    end
end

