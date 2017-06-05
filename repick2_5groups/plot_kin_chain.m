function plot_kin_chain(kinc,kinConfig, tsteps)
%PLOT_KIN_CHAIN     -plots the links of the kinematic chain defined by the
%structure CHAIN at the timesteps TSTEPS.  The kinematic chain will be
%plotted in the zero basis of link 1.
hold on
nlinks = length(kinConfig.link);
for ll = 1:nlinks
    if isempty(kinConfig.link(ll).BFvecs)
        continue
    end
    points = [kinConfig.link(ll).BFvecs,kinConfig.link(ll).BFvecs(:,1)];
    if ll>1
        path = kinpath(kinConfig.link,1,ll);
        if isempty(kinConfig.link(path(end-1)).BFvecs)
            points_prev = kinConfig.link(path(end-2)).BFvecs(:,kinConfig.link(path(end-1)).ConPt);
        else
            points_prev = kinConfig.link(path(end-1)).BFvecs(:,kinConfig.link(path(end)).ConPt);
        end
    end

    %n_pts_tot = n_pts_tot+n_bf_pts;
    for tt = tsteps
        X = [eye(3,3),zeros(3,1)]*hnode2node(kinc(tt),kinConfig,1,ll)*[points;ones(1,size(points,2))];
        if ll>1
            if isempty(kinConfig.link(path(end-1)).BFvecs)
                X = [X,[eye(3,3),zeros(3,1)]*hnode2node(kinc(tt),kinConfig,1,path(end-2))*[points_prev;1]];
            else
                X = [X,[eye(3,3),zeros(3,1)]*hnode2node(kinc(tt),kinConfig,1,path(end-1))*[points_prev;1]];
            end
        end
            
        if tt==tsteps(1)
            linespec = '-m';
        else
            linespec = '-k';
        end
        %feat_manip = 1000*YPRTransform([0,-15*pi/180,-5/180*pi],[.300,1.200,.400])*[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1]'*[X;ones(1,size(X,2))];
        feat_manip = X;
        h = plot3(feat_manip(1,:)',feat_manip(2,:)', feat_manip(3,:)', linespec, 'LineWidth', 2);
        uistack(h,'top')
    end
end