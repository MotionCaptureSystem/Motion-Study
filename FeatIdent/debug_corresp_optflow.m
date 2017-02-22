%%Debug routine for CORRESP_OPTFLOW.m
addpath D:\Users\Matt\Documents\GitHub\MotionStudy\MotionEst\Tracking

tsteps = 399:410;
path = 'C:\ShandongData2016\BatFlight_20160717\Test001';
cam = 302;

for kk = tsteps
    im_k   = imread([path,filesep,'Cam',num2str(cam),filesep,num2str(kk),'.png']);
    im_k   = rgb2gray(im_k);
    im_km1 = imread([path,filesep,'Cam',num2str(cam),filesep,num2str(kk-1),'.png']);
    im_km1 = rgb2gray(im_km1);

    if ~exist('Cam','var')
        imported = load([path,filesep,'CamStruct.mat']);
        Cam(301:341) = imported.Cam;
    end

    pt_nums = [105,141,100];

    phi_hat = reshape(Cam(cam).pts(:,kk-Cam(cam).start_frame,pt_nums),[],1) + 0.8*(reshape(Cam(cam).pts(:,kk-Cam(cam).start_frame,pt_nums),[],1)-reshape(Cam(cam).pts(:,kk-Cam(cam).start_frame-1,pt_nums),[],1));
    phi     = reshape(Cam(cam).pts(:,kk-Cam(cam).start_frame+1,pt_nums),[],1);
    phi_km1 = reshape(Cam(cam).pts(:,kk-Cam(cam).start_frame,pt_nums),[],1);

    phi_corr = corresp_optflow(phi_hat,phi,phi_km1,im_k,im_km1);

    figure
    imshow(im_k)
    hold on
    plot(phi_hat(1:2:end),phi_hat(2:2:end),'+c')
    plot(phi(1:2:end),phi(2:2:end),'.m')

    for pp = 1:length(phi_hat)/2
       plot([phi_hat(2*(pp-1)+1);phi_corr(2*(pp-1)+1)], [phi_hat(2*pp);phi_corr(2*pp)],'-y')
    end
end