function meas = create_meas_matrix(camstruct, options)
%determine the number of cameras
ncam = length(options.est.cams);
%determine the number of timesteps 
nsteps =  options.tstop - options.tstart + options.interp;
%determine the time step
dt = options.dt;

cam_cnt = 0;
if strcmp(options.est.type, 'joint')
    link = options.link;
    %determine the number of links to be estimated
    nlinks = length(link);
    %seed a measurement matrix
    meas = zeros(ncam*options.nmeas,floor(nsteps/dt));
    for cc = options.est.cams                                   %loop over the cameras
        cam_cnt = cam_cnt+1;
        time_vec = [options.tstart:dt:options.tstop]-camstruct(cc).start_frame+1+floor(119.88*camstruct(cc).sync_del);
        for ll = 1:nlinks                                       %loop over the links
            npts = size(link(ll).BFvecs,2);                     %determine the number of points on this link
            for pp = 1:npts                                     %loop over the points on this link
            point = camstruct(cc).pt_assoc{ll}(pp);
            ii = 2*(pp-1)+1;                                    %MeasInds to grab
                if isempty(point)                               %This point on this link was not seen, fill with NaN
                    keyboard
                    indx = options.nmeas*(cam_cnt-1)+link(ll).MeasInds(ii:ii+1);
                    meas(indx,:) = NaN*ones(2,nsteps);
                else                                            %Otherwize, copy the points
                    indx = options.nmeas*(cam_cnt-1)+link(ll).MeasInds(ii:ii+1);
                    meas(indx,:) = camstruct(cc).pts_sync(:,time_vec,point); 
                end
            end
        end
    end
else
    %seed a measurement matrix
    npts = length(options.pts);                     %determine the number of points
    meas = zeros(ncam*2*npts,floor(nsteps/dt));
    for cc = options.est.cams                                   %loop over the cameras
        cam_cnt = cam_cnt+1;
        time_vec = [options.tstart:dt:options.tstop]-camstruct(cc).start_frame+1+floor(119.88*camstruct(cc).sync_del);
        for pp = 1:npts                                     %loop over the points on this link
            if isempty(camstruct(cc).pts_sync)
                meas(2*npts*(cam_cnt-1)+2*(pp-1)+1:2*npts*(cam_cnt-1)+2*pp,:) = NaN*zeros(2,length(time_vec));
            elseif size(camstruct(cc).pts_sync,3)<options.pts(pp)
                meas(2*npts*(cam_cnt-1)+2*(pp-1)+1:2*npts*(cam_cnt-1)+2*pp,:) = NaN*zeros(2,length(time_vec));
            else
                meas(2*npts*(cam_cnt-1)+2*(pp-1)+1:2*npts*(cam_cnt-1)+2*pp,:) = camstruct(cc).pts_sync(:,time_vec,options.pts(pp)); 
            end
        end
    end
end
    

