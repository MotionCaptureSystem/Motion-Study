%% Inspect Stereo Triangulations

pts = [105,100,141,...
       93,87,89,91,...
       144,174,172,...
       47,54,41,56,44,58,...
       208,212,190,192,167,170,...
       203,188,166,...
       37,21,25,...
       201,187,165,...
       1,4,8];

colors = hsv(length(pts));
for pp = pts
    figure
    hold on
    for ss = 1:length(Stereo)
        if isempty(intersect(Stereo(ss).cams,[11,12,15]))
            plot3(Stereo(ss).pts(1,:,pp)',Stereo(ss).pts(2,:,pp)',Stereo(ss).pts(3,:,pp)','color',colors(find(pts==pp),:))
        end
    end
    axis equal
    title(['pt number: ',num2str(pp)])
end

%% Plot the camera space measurments
for cc = 1:length(Cam)
    figure
    hold on
    for pp = pts
        plot(Cam(cc).pts_sync(1,:,pp)',Cam(cc).pts_sync(2,:,pp)','color',colors(find(pts==pp),:))
    end
    title(['Cam ', num2str(cc)])
end