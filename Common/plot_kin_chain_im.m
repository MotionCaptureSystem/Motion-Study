function plot_kin_chain_im(kinc, kinConfig, camstruct, tsteps, varargin)
%PLOT_KIN_CHAIN_IM     -plots the links of the kinematic chain defined by the
%structure KINC using the kinematic definition in
%KINCONFIG.  CAMSTRUCT contains the calibration parameters for the camera
%in which the skeleton is to be plotted.  

if nargin>4
    flag = varargin{1};
else 
    flag = 0;
end

hold on
nlinks = length(kinConfig.link);
for ll = nlinks:-1:1
    
    if ~isempty(kinConfig.link(ll).BFvecs)
        points = [kinConfig.link(ll).BFvecs, kinConfig.link(ll).BFvecs(:,1)];%];
        n_bf_pts = size(points,2);
        for tt = tsteps
            H1 = hnode2node(kinc(tt),kinConfig,1,ll);
            %CFPlot(H1,10)
            X = [eye(3,3),zeros(3,1)]*H1*[points;ones(1,n_bf_pts)];
            
            if ~isempty(kinConfig.link(ll).ConPt)
                if ~isempty(kinConfig.link(kinConfig.link(ll).parent).BFvecs)
                    con_pt = kinConfig.link(kinConfig.link(ll).parent).BFvecs(:,kinConfig.link(ll).ConPt);
                    H = hnode2node(kinc(tt),kinConfig,1,kinConfig.link(ll).parent);
                else
                    con_pt = kinConfig.link(kinConfig.link(kinConfig.link(ll).parent).parent).BFvecs(:,kinConfig.link(kinConfig.link(ll).parent).ConPt);
                    H = hnode2node(kinc(tt),kinConfig,1,kinConfig.link(kinConfig.link(ll).parent).parent);
                end
                con_pt = [eye(3,3),zeros(3,1)]*H*[con_pt;1];
                X = [X(:,1:end-1,1),con_pt,X(:,end)];
            end
            if flag
                if ~isempty(kinConfig.link(ll).parent)
                    numbers = [kinConfig.link(ll).pt_nums,kinConfig.link(kinConfig.link(ll).parent).pt_nums(kinConfig.link(ll).ConPt)];
                else
                    numbers = kinConfig.link(ll).pt_nums;
                end
                for pp = 1:size(X,2)-1
                    text(X(1,pp)',X(2,pp)',X(3,pp)',num2str(numbers(pp)))
                end
            end
            if tt==tsteps(1)
                linespec = 'o-m';
            else
                linespec = 'o-c';
            end
            [phi, ~] = CamNetDistortion(X(:), camstruct);
            phi = reshape(phi,2,[]);
            plot(phi(1,:)',phi(2,:)', linespec)
        end
    end
end
