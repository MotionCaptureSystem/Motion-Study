function [x_k, Rt] = run_gdpm_predict(x_km1, x_km2, Rt_handle, options)

Rt = Rt_handle(1);
x_k = zeros(size(x_km1));
% use the linear velocity model for torso translation
x_k(1:3,1) = 2*x_km1(1:3)-x_km2(1:3);

%Load the GPDM and compute the remaining latent variables
load(options.bat_model)
[K invK] = computeKernel(X, theta);
[Xin Xout] = priorIO(X, segments, modelType);
[Kd invKd] = computePriorKernel(Xin, thetad, modelType(3));
simSteps = 2;
% starts at end of training sequence;
simStart = [x_km1(4:end)' x_km2(4:end)']; %inputs 2 points in case using 2nd order model
[X_pred XRand_pred] = simulatedynamics(X, segments, thetad, invKd, simSteps, simStart, modelType);

x_k(4:end) = X_pred(2,:)';
