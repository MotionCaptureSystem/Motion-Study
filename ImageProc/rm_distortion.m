function x_caltech = rm_distortion(x_p, K, fc, prin_p, skew, dist_c)
npts = length(x_p)/2;
x_caltech = NaN*ones(length(x_p),1);
for pp = 1:npts
    x_caltech(2*(pp-1)+1:2*pp) = caltech_normalize(x_p(2*(pp-1)+1:2*pp), fc, prin_p, dist_c, skew);
    x_caltech(2*(pp-1)+1:2*pp) = [1, 0, 0; 0,1, 0]*K*[x_caltech(2*(pp-1)+1:2*pp);1];
end


