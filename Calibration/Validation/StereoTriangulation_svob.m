function stereostruct = StereoTriangulation_svob(camstruct, options)
ncam      = length(options.stereo.cams);
cams      = options.stereo.cams;
npts      = length(options.stereo.pts);
pts       = options.stereo.pts;
%timesteps = options.stereo.tstart:options.stereo.dt:options.stereo.tstop;
timesteps = options.stereo.tsteps;
nsteps    = length(timesteps);
linestyle1 = options.plot.linestyle1;
plot_start = options.plot.tstart;
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
for pair = 1:npair
    for pp = 1:npts
        for kk = 1:nsteps
            %if ~isempty(intersect(pp,camstruct(pair_list(pair,1)).idin(1,pp))) && ~isempty(intersect(pp,camstruct(pair_list(pair,2)).idin(1,pp)))
                stereostruct(pair).pts(:,kk,pts(pp)) = stertridet2_svob(camstruct(pair_list(pair,1)).pts(:,timesteps(kk),pts(pp)), ...
                                                   camstruct(pair_list(pair,2)).pts(:,timesteps(kk),pts(pp)), ...
                                                   camstruct(pair_list(pair,1)),camstruct(pair_list(pair,2)));
            %else
            %    stereostruct(pair).pts = [];
            %end
            
            stereostruct(pair).cams = pair_list(pair,:);

        end
    end
end

%determine mean and std dev of stereo tri
points = zeros(3,npair);
mean_p = zeros(3,nsteps,npts);
dev = zeros(3,nsteps,npts);

mean_p_cam  = zeros(3,nsteps,npts);
dev_p_cam  = zeros(3,nsteps,npts);
for cc = 1:ncam
    [pair_inds,~] = find(pair_list==cams(cc));
    [I,~] = ind2sub(size(pair_list),pair_inds);
    points_cam = reshape([stereostruct(I').pts],3,nsteps,npts,[]);
    mean_p_cam(:,:,:,cc) = nanmean(points_cam,4);
    dev_p_cam(:,:,:,cc)  = nanstd(points_cam-repmat(mean_p_cam(:,:,:,cc),1,1,1,length(pair_inds)),0,4);
end
    
for pp = 1:npts
    for kk = 1:nsteps
        for pair = 1:npair
            points(:,pair) = stereostruct(pair).pts(:,kk,pts(pp));
        end
        points_nan = isnan(points);
        points(:,points_nan(1,:)) = [];
        mean_p(:,kk,pp) = mean(points,2);
        dev(:,kk,pp) = std(points-repmat(mean_p(:,kk),1,size(points,2)),0,2);
    end
    figure
    hold on
    plot([1:nsteps]',mean_p(1,:,pp)','-r',[1:nsteps]',(mean_p(1,:,pp)-dev(1,:,pp))','--r',[1:nsteps]',(mean_p(1,:,pp)+dev(1,:,pp))','--r');
    plot([1:nsteps]',mean_p(2,:,pp)','-b',[1:nsteps]',(mean_p(2,:,pp)-dev(2,:,pp))','--b',[1:nsteps]',(mean_p(2,:,pp)+dev(2,:,pp))','--b');
    plot([1:nsteps]',mean_p(3,:,pp)','-g',[1:nsteps]',(mean_p(3,:,pp)-dev(3,:,pp))','--g',[1:nsteps]',(mean_p(3,:,pp)+dev(3,:,pp))','--g');
    plot([1:nsteps]',dev(1,:,pp)','-r');
    plot([1:nsteps]',dev(2,:,pp)','-b');
    plot([1:nsteps]',dev(3,:,pp)','-g');
    set(gca,'FontSize',16)
    ylabel('STD Dev (mm)', 'FontSize', 16); xlabel('time (sample)','FontSize', 16); title(sprintf('Statistics of Stereo Triangulation Pt %d',pts(pp)),'FontSize',16);
    
    for cc = 1:ncam
        figure
        hold on
        plot([1:nsteps]',dev_p_cam(1,:,pp,cc)','-r');
        plot([1:nsteps]',dev_p_cam(2,:,pp,cc)','-b');
        plot([1:nsteps]',dev_p_cam(3,:,pp,cc)','-g');
        set(gca,'FontSize',16)
        ylabel('STD Dev (mm)', 'FontSize', 16); xlabel('time (sample)','FontSize', 16); title(sprintf('Statistics of Stereo Triangulation Pt %d Cam %d',pts(pp),cams(cc)),'FontSize',16);
    end
end


