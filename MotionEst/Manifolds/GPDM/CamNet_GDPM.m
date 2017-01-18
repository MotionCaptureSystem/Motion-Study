function [y_bark, Qk] = CamNet_GDPM(x_k, camstruct, options)
load(options.bat_model)
[~, invK] = computeKernel(X, theta);
%% Map X_pred back to Y_pred
%[Kpred, invKpred] = computeKernel(x_k,theta);
ky = computeKernVec(x_k(4:end)',X,theta);
q_k = Y'*invK*ky + repmat(meanData',1,size(ky,2));
q_k = [x_k(1:3,1);q_k];

nstates = length(q_k);
link    = options.link;
groups  = unique([link.Group]);
g_max   = max(groups);
nDof    = 0;
links   = [];
for gg = 1:g_max
    %links = [get_group_links(link,gg)];
    links = [links,get_group_links(link,gg)];
    %nDof = nDof + sum([link(links).nDof]);
    nDof = sum([link(links).nDof]);
    if nDof == nstates
        g_max = gg;
        break
    end
end

last_group_links = get_group_links(link,g_max);

cams = options.est.cams;

ncam = length(cams);
nmeas = ncam*length([link(links).MeasInds]);

Pi0 = [1,0,0,0;0,1,0,0];
z_hat = [0;0;1;0];
if isempty(link(links(1)).BFvecs)
    MeasStart = link(links(2)).MeasInds(1);         
else
    MeasStart = link(links(1)).MeasInds(1);            
end

%Grab a mean
y_bark = zeros(nmeas,1);
%Qk = 1*eye(length(y_bark));
Qk = zeros(length(y_bark));
%uncert = 2*[1:length(links)];
%uncert = logspace(0,3,length(links));
uncert = {[10,10,10],[10,10],[30,10],[30,30,30],[30,30],[80,50,30],[80,50,30],[80,30]};
%uncert = {[10,10,10],[3,10],[3,3,3],[3,3],[8,5,3],[8,5,3],[8,3]};
%uncert = {[1,1,1,1,1],[1],[1],[1],[1],[1],[1]};
for cc = 1:ncam         %for each camera
    Hin = invH(camstruct(cams(cc)).H);
    for ll = links
        nvecs = size(link(ll).BFvecs,2);
        H_ll = HTransform(q_k(1:sum([link(1:ll).nDof]),1),link);
        %H_ll = HTransform(x_k,link);
        for pp = 1:nvecs
            x_lpi = [link(ll).BFvecs(:,pp);1];
            %Determine predicted range to point
            lambda = z_hat'*Hin*H_ll*x_lpi; 
            %Determine Sensor Model Jacobian
            ndx = nmeas/ncam*(cc-1)+link(ll).MeasInds(2*pp-1:2*pp)-MeasStart+1;
            y_bark(ndx) = 1/lambda*Pi0*[camstruct(cams(cc)).K,[0;0;0];0,0,0,1]*Hin*H_ll*x_lpi;
            
            link_inds = links == ll;
            Qk(ndx,ndx) = uncert{ll}(pp)* eye(length(ndx));

        end
    end
end