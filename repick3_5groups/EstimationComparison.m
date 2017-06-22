%% Compare Estimation Techniques
clear all
close all

load('1dofPhal.mat','eststruct');
EstStruct(1) = eststruct;
load('1dofPhal2_noWrist.mat','eststruct');
EstStruct(2) = eststruct;
load('2dofPhal_noWrist2.mat','eststruct');
EstStruct(3) = eststruct;
load('noWrist.mat','eststruct');
EstStruct(4) = eststruct;
load('precomp.mat','eststruct');
EstStruct(4) = eststruct;
% load('EstStructBugFix_IID.mat');
% EstStruct(3).options = options;
% EstStruct(3).ukf = ukf;
% EstStruct(3).kinc = kinc;
load('precomp.mat','Cam','options')

fprintf('data loaded ...\n')

linetypes = {'-','--','-.',':','+-'};
fs_c = 120;
%%
Plot_Script_JS_multi_oneplot(Cam,EstStruct,linetypes,options)
reproj_error(Cam, EstStruct, options, linetypes)