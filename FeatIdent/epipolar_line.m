function ep_line = epipolar_line(H1, H2, phi1)
%EPIPOLAR_LINE      -Computes the epipolar line in camera 2 based on the
%homogenous transformations between camera 1 and camera 2 and the retinal
%coordinates of a point in camera 1. 

o1 = H1(1:3,4);
o2 = H2(1:3,4);
R1 = H1(1:3,1:3);
R2 = H2(1:3,1:3);

el = o2-o1;
ep = [eye(3),zeros(3,1)]*invH(H2)*[o1;1];
ep_phi = ep/ep(3);

for pp = 1:size(phi1,2)
    n_hat = cross(el, R1*phi1(:,pp))/norm(cross(el, R1*phi1(:,pp)));
    % pts = [o1,o1+0.4*n_hat];
    % plot3(pts(1,:)',pts(2,:)',pts(3,:)', '-m')

    le = R2'*cross(n_hat,R2(1:3,3));
    le_phi = le/norm(le);
    % pts = [o2,o2+0.4*R2*le_phi];
    % plot3(pts(1,:)',pts(2,:)',pts(3,:)', '-m')

    m = le_phi(2)/le_phi(1);
    b = ep_phi(2)-m*ep_phi(1);

    ep_line(pp,:) = [m b];
end



