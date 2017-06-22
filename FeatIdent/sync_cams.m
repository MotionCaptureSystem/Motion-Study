function camstruct = sync_cams(camstruct, options)


y = 'y';%input('Were the cameras hardware synced? [y/n]','s');
if strcmpi(y,'y')
    for cc = 1:length(camstruct)
        camstruct(cc).pts_sync = camstruct(cc).pts_rect;
        camstruct(cc).sync_del = 0;
    end
else
    if isfield(camstruct,'sync_del')
        y = input('Sync Data Already Exists.  Do you want to recompute the delays?:','s');
        if strcmp(y, 'y')
            [options, camstruct] = audiosync(camstruct, options);
        end
    else
    [options, camstruct] = audiosync(camstruct, options);
    [camstruct] = subframe_sync(camstruct, options);
    end
end
