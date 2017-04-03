function phi_corr = corresp_optflow(phi_hat,phi,phi_km1,im_km1,im_k)
%finds the closest measurement to each guess. 
%phi_hat    -image space estimate of feature location [ncam*npts*2,1]
%phi        -uncorresponded list of potenttial measurements [ncam*npts*2,1]
%phi_km1    -previous measurment location
%im_km1     -image at the previous timestep
%im_k       -current image

%determine the number of predicted points
npts = length(phi_hat)/2;
%initialize the vector for saving corresponded points
phi_corr = 2*zeros(npts,1);
%initialize the distance vector
dist = zeros(npts,1);
%initialize the correspondence index vector.
numbers = zeros(npts,1);

%convert the images to double class
h = fspecial('gaussian',5,1);
G{1} = double(imfilter(histeq(im_km1),h));
G{2} = double(imfilter(histeq(im_k),h));
% G{1} = double(histeq(im_km1));
% G{2} = double(histeq(im_k));
%format the predicted locations appropriately 
pts = reshape(phi_hat,2,[]);
%format the current locations appropriately
pts_km1 = reshape(phi_km1,2,[]);
%compute the new predicted location using optical flow
[y,x]=mttrack(G,[pts_km1(2,:)',zeros(size(pts_km1,2),1)],[pts_km1(1,:)',zeros(size(pts_km1,2),1)],[pts(2,:)',zeros(size(pts,2),1)],[pts(1,:)',zeros(size(pts,2),1)]);
%[y,x]=mttrack(G,[pts_km1(2,:)',zeros(size(pts_km1,2),1)],[pts_km1(1,:)',zeros(size(pts_km1,2),1)],[pts_km1(2,:)',zeros(size(pts_km1,2),1)],[pts_km1(1,:)',zeros(size(pts_km1,2),1)]);
%reformat the new prediction
phi_hat = [x(:,2)';y(:,2)'];
phi_hat = reshape(phi_hat,[],1);
%hold on;
%plot(phi(1:2:end),phi(2:2:end),'*m')
%compute the difference between the pp^th estimate and all candiates
for pp = 1:npts
    %compute the difference between the pth prediction and all candidate
    %features.
    delta = repmat(phi_hat(2*(pp-1)+1:2*pp),length(phi)/2,1) - phi;
    %compute the square sum of distances
    delta_sq = sum(reshape(delta.*delta,2,[]));
    %find the minimum distance and its corresponding index
    [dist(pp),numbers(pp)] = min(delta_sq);
    %if the estimate is within the allowable threshold, save the point.  If
    %not, store NaN.
    if dist(pp)<10^2
        phi_corr((2*(pp-1)+1:2*pp),1) = phi(2*(numbers(pp)-1)+1:2*numbers(pp));
    else
        phi_corr((2*(pp-1)+1:2*pp),1) = [NaN;NaN];
    end
end

        