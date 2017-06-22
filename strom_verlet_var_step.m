function [x_k] = strom_verlet_var_step(x_km1, x_km2)
%MANIFOLD_DYN   -Computes the state prediction X_K using the identified
%manifold potential computed by dVzdx_handle.

%x_km1          -state vector at t = k-1
%x_km2          -state vector at t = k-2
%dVzdx_handle   -function handle which computes the manifold potential
%                The function must be of the form dVzdx_handle(xk, Xk)
%                where,
%                   xk  = state at which potential should be computed
%                   Xk  = all of the sample points for manifold estimation
%                dVzdx_handle should be declared as,
%                   dVzdx_handle = @(xk)  manfold_function(xk,Xk);
%                where MANFOLD_FUNCTION is the name of the function that
%                computes the potential.
%options        -options structure from MotionStudy.  Must contain
%                options.empman.ep = epsilon value for potential
%                options.fs = sampling frequency

%grab the constants from the options structure
dt_km2 = 1/100;
%ep = options.empman.ep;
%m = 0.001;
%g = [0;0;9.81];     %gravity vector
sub_samp = 10;
dt_km1 = dt_km2/sub_samp;
%compute the state prediction
history = zeros(length(x_km1),sub_samp+1);
history(:,1) = x_km1;
for ii = 1:sub_samp
    %x_k = (g-(1/m)*(1/ep^2)*dVzdx_handle(x_km1))*(dt_km1+dt_km2)/2*dt_km1 + x_km1 + (x_km1-x_km2)*(dt_km1/dt_km2);
    x_k = [1;0;0]*(dt_km1+dt_km2)/2*dt_km1 + x_km1 + (x_km1-x_km2)*(dt_km1/dt_km2);
    history(:,ii+1) = x_k;
    x_km2 = x_km1;
    x_km1 = x_k;
    dt_km2 = dt_km1;
end

figure
hold on
plot3(history(1,:)',history(2,:)',history(3,:)','.-m')

figure
hold on
plot(history(1,:)','.-r')
plot(history(2,:)','.-g')
plot(history(3,:)','.-m')
