cams = options.est.cams;

clear M
figure
for cc = cams
    
    for kk = 3:length(eststruct.kinc)
        clf
        im = imread([options.path,filesep,'Cam',num2str(cc),filesep,num2str(kk+options.tstart-3),'.png']);
        imshow(im)
        hold on
        
        plot_kin_chain_im(kinc,options,Cam(cc),kk)
        pt_nums = [options.link.pt_nums];
        out_range = pt_nums>size(Cam(cc).pts,3);
        if any(out_range)
            pt_nums(out_range) = [];
        end
        pts_orig = reshape(Cam(cc).pts(:,options.tstart-Cam(cc).start_frame-1+kk-1,pt_nums),2,[]);
        plot(pts_orig(1,:)',pts_orig(2,:)','c+','MarkerSize',2)
        drawnow
        M(kk-2) = getframe(gcf);
    end

    vidObj = VideoWriter(['SkelReproj_Cam',num2str(cc),'.avi']);
    vidObj.FrameRate = 10;
    open(vidObj);
    writeVideo(vidObj,M);
    close(vidObj);
    
    clear vidObj
    clear M
end