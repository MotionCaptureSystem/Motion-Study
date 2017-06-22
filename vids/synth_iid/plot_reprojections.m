im = imread('400.png');

load CamStruct.mat

load AutoCorr_All_Track
z_gg_all_track = z_gg_auto;

load Corresp_OptFlow_RecTrack
z_gg_rec_track = z_gg_auto;
ncam = 9;
npts = size(z_gg_rec_track,1)/2/ncam;
nsteps = size(z_gg_rec_track,2);

pts = [4,1,5,6,8,7,10,9,14,13,17];
w = 8;
figure

for kk = 3+w:nsteps
    cam_time = 390+floor(Cam(18).sync_del*120)+kk-2;
    imshow(imread(['..', filesep, 'Cam318', filesep, num2str(cam_time), '.png']))
    hold on 
    cc = 6;
    colors = hsv(npts);
    for pp = 1:npts
        h1 = plot(z_gg_rec_track(2*npts*(cc-1)+2*(pp-1)+1,kk-w:kk)',z_gg_rec_track(2*npts*(cc-1)+2*pp,kk-w:kk)','-','color',colors(pp,:),'linewidth', 1.5);
        h2 = plot(z_gg_all_track(2*npts*(cc-1)+2*(pp-1)+1,kk-w:kk)',z_gg_all_track(2*npts*(cc-1)+2*pp,kk-w:kk)','--.','color',colors(pp,:),'linewidth', 1.5, 'markersize',10);
        h3 = plot(Cam(18).pts(1,cam_time-Cam(18).start_frame+1-w:cam_time-Cam(18).start_frame+1,pts(pp)),...
                    Cam(18).pts(2,cam_time-Cam(18).start_frame+1-w:cam_time-Cam(18).start_frame+1,pts(pp)),'o','color',colors(pp,:));
    end
    axis([509 1056 128 503])
    legend([h1,h2,h3],{'SpaRe','Standard', 'Original'})
    M(:,:,kk-2-w) = getframe;
    %delete(gcf)
end

movie(M)