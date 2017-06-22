%%data prep for Aishwarya
ll = 4;
cam = 301;

%get all the points into a matrix
pts_all = zeros([size(Stereo(1).pts),length(Stereo)]);
for ss = 1:length(Stereo)
    pts_all(:,:,:,ss) = Stereo(ss).pts;
end

%cut out the points
pts_forearm_all = pts_all(:,:,Cam(cam).pt_assoc{ll},:);
median_forearm_all = nanmedian(pts_forearm_all,4);

%computer intertial coordinates from the timeseries of homogeneous
%transforms computed by the state estimate. 
pts_rigid_0 = zeros(3,length(kinc)-2,length(Cam(cam).pt_assoc{ll}));
for kk = 3:length(kinc)
    H0ll         = hnode2node(kinc(kk),options,1,ll);
    pts_rigid_0(:,kk-2,:) = reshape([eye(3),zeros(3,1)]*H0ll*[synthConfig.link(ll).BFvecs;ones(1,size(synthConfig.link(ll).BFvecs,2))],3,1,[]);
end

%%plot the points in the inertial frame
figure
hold on
for pp = 1:size(median_forearm_all,3)
    plot3(median_forearm_all(1,:,pp)',median_forearm_all(2,:,pp)',median_forearm_all(3,:,pp)','-')  
    plot3(pts_rigid_0(1,:,pp)',pts_rigid_0(2,:,pp)',pts_rigid_0(3,:,pp)','+')
end
legend('Stereo Tri','Rigid Body')


pts = zeros([size(Stereo(1).pts,1),size(Stereo(1).pts,2),length(Cam(cam).pt_assoc{ll})]);
pts_rigid = zeros([size(Stereo(1).pts,1),size(Stereo(1).pts,2),length(Cam(cam).pt_assoc{ll})]);
for kk = 3:size(median_forearm_all,2)+2
    H0c                 = hnode2node(kinc(kk),options,1,ll-1);
    Hc_cp1              = hnode2node(kinc(kk),options,ll,ll);
    pts_kk              = reshape(median_forearm_all(:,kk-2,:),3,[]);
    pts(:,kk-2,:)       = reshape([eye(3),zeros(3,1)]*invH(H0c)*[pts_kk;ones(1,size(pts_kk,2))],3,1,[]);
    pts_rigid(:,kk-2,:) = reshape([eye(3),zeros(3,1)]*Hc_cp1*[synthConfig.link(ll).BFvecs;ones(1,size(synthConfig.link(ll).BFvecs,2))],3,1,[]);
end

%pts = pts-repmat(pts(:,:,2),1,1,size(pts,3));
figure
hold on
colors = hsv(size(pts,3));
for pp = 1:size(pts,3)
    plot3(pts(1,:,pp)',pts(2,:,pp)',pts(3,:,pp)','+','color',colors(pp,:))
    plot3(pts_rigid(1,:,pp)',pts_rigid(2,:,pp)',pts_rigid(3,:,pp)','o','color',colors(pp,:))
end
axis equal