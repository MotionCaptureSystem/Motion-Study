function cost = fcost2(camstruct, options, delta_vec)
%Grab Required Options

npts = length(options.ba_pts);
pts = options.ba_pts;
timesteps = options.ba_tsteps;
nsteps = length(options.ba_tsteps);
nparam = length(delta_vec);
cams = options.cams;
ncam = length(cams);

%delta_vec contains 3D point locations and camera parameters
%delta_vec = [delta_vec_pts; delta_vect_param];

%points are partitioned by point number, then by timestep
%delta_vec_pts = [delta_vec_x1t1; delta_vec_x2t1; ... delta_vec_xntk];

%params are partitioned by then pos, then orientation, then camera #
%delta_vec_param = [r_c1; ... r_cm; d_c1; ... d_cm];

delta_vec_pts = delta_vec(1:3*npts*nsteps);
X = reshape(delta_vec_pts, 3*npts,[]);
delta_vec_param = delta_vec(3*npts*nsteps+1:end);
delta_vec_rot = delta_vec_param(1:3*ncam);
delta_vec_pos = delta_vec_param(3*ncam+1:end);

%Modify extrinsics of camera setup
for cc = 1:ncam
    %camstruct(cams(cc)).H = camstruct(cams(cc)).H*[eye(3),delta_vec_pos(3*(cc-1)+1:3*cc,1);zeros(1,3),1];
    camstruct(cams(cc)).H = camstruct(cams(cc)).H*[caltech_rodrigues(delta_vec_rot(3*(cc-1)+1:3*cc,1)),delta_vec_pos(3*(cc-1)+1:3*cc,1);zeros(1,3),1];
    Hin = invH(camstruct(cams(cc)).H);
    camstruct(cams(cc)).T = Hin(1:3,4);
    camstruct(cams(cc)).om = caltech_rodrigues(Hin(1:3,1:3));
    
    if norm(camstruct(cams(cc)).om) == 0;
        camstruct(cams(cc)).om = [2*pi;0;0];
    end
    
end


for kk = 1:nsteps
        [phi_est(:,kk),~]         = CamNet(X(:,kk), camstruct);
end

phi_meas = zeros(2*npts*ncam,nsteps);
for cc = 1:ncam
    phi_meas(2*(cc-1)*npts+1:2*cc*npts,:) = reshape(camstruct(cams(cc)).pts(:,timesteps,:),[],size(timesteps,2));
end

phi_error = reshape(phi_est-phi_meas,[],1);

occlusions = isnan(phi_error);
phi_error(occlusions) = [];

cost = phi_error'*phi_error;