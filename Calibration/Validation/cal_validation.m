clear all
close all

%% Run Initialization Script for Desired Data Set
Init_cal_val_20160614

%% Plot Rig Extrinsics
cnt =  0;
for cc = options.cams
    cnt = cnt+1;
    H(:,:,cnt) = Cam(cc).H;
end
H(:,:,cnt+1) = eye(4);

figure
CFPlot(H,0.01);
axis equal

%% Remove Distortion from Points
for cc = options.cams
    for kk = 1:size(Cam(cc).pts_dist ,2) %x_p, K, fc, prin_p, skew, dist_c
        Cam(cc).pts(:,kk) = rm_distortion(Cam(cc).pts_dist(:,kk),Cam(cc).K_dist,[Cam(cc).K_dist(1,1) Cam(cc).K_dist(2,2)], Cam(cc).K_dist(1:2,3)', Cam(cc).K_dist(2,1), Cam(cc).kc);
    end
end

%% Run the Bundle Adjustment
%Cam = bundle_adjust_pixel2(Cam, options);

%% Run Stereo Triangulation 
EstPoints.ster = StereoTriangulation_svob(Cam, options);

%% Plot Stereo Triangulations Using Different Pairs of Cameras
plot_cams = options.cams;
all_pairs = reshape([EstPoints.ster.cams],2,[]);
pair_inds = find(sum(ismember(all_pairs,plot_cams))==2);

colors = hsv(length(pair_inds));
npts = length(options.pts);
figure
hold on
cnt = 0;
for pair = pair_inds
    cnt = cnt+1;
    if ~isempty(EstPoints.ster(pair).pts)
        for pp = 1:npts
            plot3(EstPoints.ster(pair).pts(1,:,options.pts(pp))', EstPoints.ster(pair).pts(2,:,options.pts(pp))', EstPoints.ster(pair).pts(3,:,options.pts(pp))','.','Color', colors(cnt,:))
        end
    end
end
%legend('PT 2 Cams 1 and 2', 'PT 2 Cams 1 and 3', 'PT 2 Cams 2 and 3')
H = reshape([Cam(plot_cams).H],4,4,[]);
CFPlot(H, 0.1)
axis equal
set(gca, 'FontSize', 16, 'CameraPosition', [0, 0, 0])
xlabel('x (mm)', 'FontSize', 16)
ylabel('y (mm)', 'FontSize', 16)
zlabel('z (mm)', 'FontSize', 16)
title ('Stereo Triangulation', 'FontSize', 18)