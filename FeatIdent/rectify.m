function camstruct = rectify(camstruct,cams)

for c = cams
    fprintf('Rectifying Points from Camera %d ... \n',c)
    camstruct(c).pts_rect = NaN*zeros(size(camstruct(c).pts));
    pts = 1:size(camstruct(c).pts,3);
    for pp = pts
        for kk = 1:size(camstruct(c).pts,2)
            %if the point is occluded the value will be NaN
            point = camstruct(c).pts(:,kk,pp);
            point_ud = rm_distortion(point, camstruct(c).K_dist, camstruct(c).fc_dist,...
                       camstruct(c).cc_dist, camstruct(c).alpha_c_dist, camstruct(c).kc_dist);
            camstruct(c).pts_rect(:,kk,pp) = point_ud;
        end
    end
end