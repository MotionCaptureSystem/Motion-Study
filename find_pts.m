%% find points
pt_nums = [1,4,8,19,22,25,37,165,166,187,188,201,202,203];

cnt = 0;
num_pairs = zeros(length(pt_nums),size(Stereo(1).pts,2));
for pp = pt_nums
    cnt = cnt+1;
    pts_all = zeros(3,size(Stereo(1).pts,2),length(Stereo));
    for ss = 1:length(Stereo)
        pts_all(:,:,ss) = Stereo(ss).pts(:,:,pp);
    end
    num_pairs(cnt,:) = sum(~isnan(pts_all(1,:,:)),3);
    labels{cnt} = num2str(pp);
end

figure
plot(num_pairs')
legend(labels{:})

table_vals = [pt_nums',min(num_pairs,[],2),max(num_pairs,[],2)]