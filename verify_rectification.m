%% Verify Rectification

pts = [100,105,141,93,87,58,56,54,44,41,47,25,21,37,8,4,1,144,174,170,192,212,167,190,208,166,188,203,165,187,201];
npts = length(pts);
colors = hsv(npts);

for cc = 1:length(Cam)

    if ~isempty(Cam(cc).pts)
        figure
        hold on
        for pp = pts
            if pp<=size(Cam(cc).pts,3)
                plot(Cam(cc).pts(1,:,pp)',Cam(cc).pts(2,:,pp)','color',colors(find(pts==pp),:))
            end
            if pp<=size(Cam(cc).pts_rect,3)
                plot(Cam(cc).pts_rect(1,:,pp)',Cam(cc).pts_rect(2,:,pp)','-.','color',colors(find(pts==pp),:))
            end
        end
        title(['Cam ', num2str(cc)])
    end
end