%clear all
%close all

%% Run Initialization Script for Desired Data Set
Init_cal_val_20160524

%% Run the Bundle Adjustment
Cam = bundle_adjust_pixel2(Cam, options);

%% Run Stereo Triangulation 
EstPoints.ster = StereoTriangulation_svob(Cam, options);

