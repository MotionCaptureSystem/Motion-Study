function phi_corr = corresp(z_hat,phi)
%finds the closest measurement to each guess
npts = length(z_hat)/2;
phi_corr = zeros(length(z_hat),1);
dist = zeros(npts,1);
numbers = zeros(npts,1);
for pp = 1:npts
    %compute the difference between the pp^th estimate and all candiates
    delta = repmat(z_hat(2*(pp-1)+1:2*pp),length(phi)/2,1) - phi;
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
        