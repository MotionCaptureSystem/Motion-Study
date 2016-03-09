function plot_ret_plane_inert(H,phi)
%PLOT_RET_PLANE_INERT   -plots the retinal plane of the camera and the
%point phi in inertial cooridnates


CFPlot(H,0.2)

ret_plane = H(1:3,1:4)*[-1,-1,1,1,-1;-1,1,1,-1,-1;1,1,1,1,1;1,1,1,1,1];
plot3(ret_plane(1,:)',ret_plane(2,:)',ret_plane(3,:)')
axis equal

phi_0 = H(1:3,1:4)*[phi;ones(1,size(phi,2))];
plot3(phi_0(1),phi_0(2),phi_0(3),'+r')