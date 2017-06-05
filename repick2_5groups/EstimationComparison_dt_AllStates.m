%% Compare Estimation Techniques
clear all
close all

EstStruct(1) = load('EstStructBugFix_AllStates.mat');
EstStruct(1).options.dt = 1;
EstStruct(2) = load('EstStruct_AllStates_dt2.mat');
EstStruct(3) = load('EstStruct_AllStates_dt3.mat');
EstStruct(4) = load('EstStruct_AllStates_dt4.mat');
EstStruct(5) = load('EstStructBugFix_CondInd.mat');
EstStruct(5).options.dt = 1;
EstStruct(6) = load('EstStruct_CondInd_dt2.mat');
EstStruct(7) = load('EstStruct_CondInd_dt3.mat');
EstStruct(8) = load('EstStruct_CondInd_dt4.mat');

for ss = 1:length(EstStruct)
    EstStruct(ss).ukf.X = EstStruct(ss).ukf.X(:,2:end);
    EstStruct(ss).ukf.Features = EstStruct(ss).ukf.Features(:,2:end);
    EstStruct(ss).kinc = EstStruct(ss).kinc(2:end);
end

load('CamStruct.mat')
Cam(cams(1):cams(2)) = Cam;

Plot_Script_JS_diffdt(Cam,EstStruct)
%reproj_error(Cam, EstStruct, EstStruct(1).options)