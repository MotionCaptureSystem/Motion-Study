function cost = fcost(camstruct, options, delta_vec)
%Grab Required Options

npts = length(options.ba_pts);
pts = options.ba_pts;
timesteps = options.ba_tsteps;
nsteps = length(options.ba_tsteps);
nparam = length(delta_vec);
cams = options.cams;
ncam = length(cams);

delta_vec_rot = delta_vec(1:nparam/2);
delta_vec_pos = delta_vec(nparam/2+1:nparam);
%delta_vec_rot = delta_vec;
%delta_vec_pos = delta_vec;
%delta_vec_pos = zeros(3*ncam,1);

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

%determine pairs of cameras
npair = 0;
pair_list = [];
for ii = 1:ncam-1
    npair = npair + ii;
    for jj = ii+1:ncam
        pair_list = [pair_list; cams(ii),cams(jj)];
    end
end

%Determine stereo reconstructions of each pair of cams with new extrinsics
X_ster = zeros(3*npts, nsteps, npair);
phi_ster = zeros(2*npts*ncam,nsteps,npair);

for pair = 1:npair
    for kk = 1:nsteps
        for pp = 1:npts
            X_ster(3*(pp-1)+1:3*pp,kk,pair) = stertridet2(camstruct((pair_list(pair,1))).pts(:,timesteps(kk),pts(pp)), ...
                                               camstruct((pair_list(pair,2))).pts(:,timesteps(kk),pts(pp)), ...
                                               camstruct((pair_list(pair,1))),camstruct((pair_list(pair,2))));
            [phi_ster(:,kk,pair),~]         = CamNet(X_ster(:,kk,pair), camstruct);
            
        end
    end
end

phi_meas = zeros(2*npts*ncam,nsteps);
for cc = 1:ncam
    phi_meas(2*(cc-1)*npts+1:2*cc*npts,:) = reshape(camstruct(cams(cc)).pts(:,timesteps,:),[],size(timesteps,2));
end

phi_error = zeros(2*npts*ncam,nsteps,npair);

%for pair = 1:npair
phi_error = (phi_ster-repmat(phi_meas,1,1,npair)).*(phi_ster-repmat(phi_meas,1,1,npair));
%end

occlusions = isnan(phi_error);
phi_error(occlusions) = 0;

if any(isnan(phi_error))
    keyboard
end
w1 = 0;
w2 = 0;
%cost = sum(sum(sum(phi_error)))+w*(delta_vec'*delta_vec);
%cost = sum(sum(sum(phi_error)))+w2*(delta_vec_pos'*delta_vec_pos);
cost = sum(sum(sum(phi_error)))+w1*(delta_vec_rot'*delta_vec_rot)+w2*(delta_vec_pos'*delta_vec_pos);