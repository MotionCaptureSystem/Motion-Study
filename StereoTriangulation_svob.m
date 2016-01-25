function stereostruct = StereoTriangulation_svob(camstruct, options)
ncam      = length(options.stereo.cams);
cams      = options.stereo.cams;
npts      = length(options.stereo.pts);
pts       = options.stereo.pts;
timesteps = options.stereo.tstart:options.stereo.dt:options.stereo.tstop;
nsteps    = length(timesteps);
linestyle1 = options.plot.linestyle1;
plot_start = options.stereo.tstart;
fs = 120;
%% Perform Stereo Triangulation for Comparision
npair = 0;
pair_list = [];
for ii = 1:ncam-1
    npair = npair + ii;
    for jj = ii+1:ncam
        pair_list = [pair_list; cams(ii),cams(jj)];
    end
end

%Determine stereo reconstructions of each pair of cams with new extrinsics
stereostruct = struct([]);
for pair = [1:npair]
    for pp = 1:npts
        for kk = 1:nsteps
            %if ~isempty(intersect(pp,camstruct(pair_list(pair,1)).idin(1,pp))) && ~isempty(intersect(pp,camstruct(pair_list(pair,2)).idin(1,pp)))
                camstruct1 = camstruct(pair_list(pair,1));
                camstruct2 = camstruct(pair_list(pair,2));
                stereostruct(pair).pts(:,kk,pts(pp)) = stertridet2_svob(camstruct1.pts_sync(:,timesteps(kk)-camstruct1.start_frame+1+floor(camstruct1.sync_del*fs),pts(pp)), ...
                                                   camstruct2.pts_sync(:,timesteps(kk)-camstruct2.start_frame+1+floor(camstruct2.sync_del*fs),pts(pp)), ...
                                                   camstruct1,camstruct2);
            
            stereostruct(pair).cams = pair_list(pair,:);
        end
    end
end

% %determine mean and std dev of stereo tri
% points = zeros(3,npair);
% mean_p = zeros(3,nsteps,npts);
% dev = zeros(3,nsteps,npts);
% 
% mean_p_cam  = zeros(3,nsteps,npts);
% dev_p_cam  = zeros(3,nsteps,npts);
% for cc = 1:ncam
%     [pair_inds,~] = find(pair_list==cams(cc));
%     points_cam = reshape([stereostruct(pair_inds').pts],3,nsteps,npts,[]);
%     mean_p_cam(:,:,:,cc) = nanmean(points_cam,4);
%     dev_p_cam(:,:,:,cc)  = nanstd(points_cam-repmat(mean_p_cam(:,:,:,cc),1,1,1,length(pair_inds)),0,4);
% end
%     
% for pp = 1:npts
%     for kk = 1:nsteps
%         for pair = 1:npair
%             points(:,pair) = stereostruct(pair).pts(:,kk,pts(pp));
%         end
%         points_nan = isnan(points);
%         points(:,points_nan(1,:)) = [];
%         mean_p(:,kk,pp) = mean(points,2);
%         dev(:,kk,pp) = std(points-repmat(mean_p(:,kk),1,size(points,2)),0,2);
%     end
%     figure
%     hold on
% %     plot([1:nsteps]',mean_p(1,:,pp)','-r',[1:nsteps]',(mean_p(1,:,pp)-dev(1,:,pp))','--r',[1:nsteps]',(mean_p(1,:,pp)+dev(1,:,pp))','--r');
% %     plot([1:nsteps]',mean_p(2,:,pp)','-b',[1:nsteps]',(mean_p(2,:,pp)-dev(2,:,pp))','--b',[1:nsteps]',(mean_p(2,:,pp)+dev(2,:,pp))','--b');
% %     plot([1:nsteps]',mean_p(3,:,pp)','-g',[1:nsteps]',(mean_p(3,:,pp)-dev(3,:,pp))','--g',[1:nsteps]',(mean_p(3,:,pp)+dev(3,:,pp))','--g');
%     plot([1:nsteps]',dev(1,:,pp)'*1500,'-r');
%     plot([1:nsteps]',dev(2,:,pp)'*1500,'-b');
%     plot([1:nsteps]',dev(3,:,pp)'*1500,'-g');
%     set(gca,'FontSize',16)
%     ylabel('STD Dev (mm)', 'FontSize', 16); xlabel('time (sample)','FontSize', 16); title(sprintf('Statistics of Stereo Triangulation Pt %d',pts(pp)),'FontSize',16);
%     
%     for cc = 1:ncam
%         figure
%         hold on
%         plot([1:nsteps]',dev_p_cam(1,:,pp,cc)'*1500,'-r');
%         plot([1:nsteps]',dev_p_cam(2,:,pp,cc)'*1500,'-b');
%         plot([1:nsteps]',dev_p_cam(3,:,pp,cc)'*1500,'-g');
%         set(gca,'FontSize',16)
%         ylabel('STD Dev (mm)', 'FontSize', 16); xlabel('time (sample)','FontSize', 16); title(sprintf('Statistics of Stereo Triangulation Pt %d Cam %d',pts(pp),cams(cc)),'FontSize',16);
% 
%     end
%     
% end
% 
% 
%         

%% Plot Stereo Triangulations Using Different Pairs of Cameras
figure
hold on
for pair = 1:npair
    if ~isempty(stereostruct(pair).pts)
        for pp = 1:npts
            %plot3(stereostruct(pair).pts(1,options.ba_tsteps,pts(pp))', stereostruct(pair).pts(2,options.ba_tsteps,pts(pp))', stereostruct(pair).pts(3,options.ba_tsteps,pts(pp))','.r')
            plot3(stereostruct(pair).pts(1,:,pts(pp))', stereostruct(pair).pts(2,:,pts(pp))', stereostruct(pair).pts(3,:,pts(pp))',linestyle1{pp})
        end
    end
end
%legend('PT 2 Cams 1 and 2', 'PT 2 Cams 1 and 3', 'PT 2 Cams 2 and 3')
H = reshape([camstruct(cams).H],4,4,[]);
CFPlot(H, 0.1)
axis equal
set(gca, 'FontSize', 16, 'CameraPosition', [0, 0, 0])
xlabel('x (mm)', 'FontSize', 16)
ylabel('y (mm)', 'FontSize', 16)
zlabel('z (mm)', 'FontSize', 16)
title ('Stereo Triangulation', 'FontSize', 18)

%----------------------------STERTRIDENT2_SVOB-----------------------------
function x = stertridet2_svob(phi1, phi2, camstruct1,camstruct2)
%Pmat1 = camstruct1.Pmat;
%Pmat2 = camstruct2.Pmat;
% [R1,Q1] = qr(Pmat1);
% [R2,Q2] = qr(Pmat2);
H1 = camstruct1.H;
H2 = camstruct2.H;
K1 = camstruct1.K;
K2 = camstruct2.K;
%Determine Vector from o1 to o2 in world frame.
P1= H1(1:3,4);
P2 = H2(1:3,4);
T = P2-P1;

%determine unit vectors to observed by cameras in world frame
 u1 = H1(1:3,1:3)*(K1\[phi1;1])/norm((K1\[phi1;1]));
 u2 = H2(1:3,1:3)*(K2\[phi2;1])/norm((K2\[phi2;1]));
%u1 = Pmat1(1:3,1:3)^-1*[phi1;1]/norm(Pmat1(1:3,1:3)^-1*[phi1;1]);
%u2 = Pmat2(1:3,1:3)^-1*[phi2;1]/norm(Pmat2(1:3,1:3)^-1*[phi2;1]);

%Determine Plane parallel to both lines. 
N_hat = cross(u1,u2);

%Project vector between coordinate frames onto normal vector (i.e. the distance between planes)
d_vec = T'*N_hat*N_hat;
%dotN_hat_T = 
%d_vec = d;

T_til = T-d_vec;

gamma = acos(u1'*u2/(norm(u1)*norm(u2)));
beta  = pi-acos(T_til'*u2/(norm(T_til)*norm(u2)));
alpha = acos(T_til'*u1/(norm(T_til)*norm(u1)));
b = norm(T_til)/sin(gamma)*sin(beta)*u1+P1;
a = norm(T_til)/sin(gamma)*sin(alpha)*u2+P2;

%d_vec_new = b-(a+T);
x = b/2+a/2;


