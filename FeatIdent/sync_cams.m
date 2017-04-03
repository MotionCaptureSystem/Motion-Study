function camstruct = sync_cams(camstruct)

for cc = 1:length(camstruct)
    camstruct(cc).pts_sync = camstruct(cc).pts_rect;
    camstruct(cc).sync_del = 0;
end