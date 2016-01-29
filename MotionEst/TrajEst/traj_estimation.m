
%% Run Initialization Script for Desired Data Set
Init_Flight_BatFlight_20150724_test004_wrist
%Init_Flight_MotionEst_Synth
fprintf('Data Loaded from %s...\n',options.path)

%StereoTriangulation_svob(Cam,options);
%% Run UKF 
tic
EstPoints.filt = run_ukf(Cam, options);
toc
%% Generate Plots of Trajectory Estimates
Plot_Script_JS

%% Compute Reprojection Error Plots
reproj_error(Cam,EstPoints,options);