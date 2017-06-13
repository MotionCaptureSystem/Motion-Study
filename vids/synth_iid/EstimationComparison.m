%% Compare Estimation Techniques
clear all
%close all

load('EstStructSynth2.mat');
EstStruct(1).ukf = ukf;
EstStruct(2).kinc = kinc;

load('std.mat','eststruct');
EstStruct(2) = eststruct;
EstStruct(2).ukf.X = EstStruct(2).ukf.X(:,3:end);
EstStruct(2).kinc = EstStruct(2).kinc(3:end);
load('iid.mat','eststruct');
EstStruct(3) = eststruct;
EstStruct(3).ukf.X = EstStruct(3).ukf.X(:,3:end);
EstStruct(3).kinc = EstStruct(3).kinc(3:end);
load('SpaRe.mat','eststruct');
EstStruct(4) = eststruct;
EstStruct(4).ukf.X = EstStruct(4).ukf.X(:,3:end);
EstStruct(4).kinc = EstStruct(4).kinc(3:end);

load('SpaRe.mat','Cam','options')

fprintf('data loaded ...\n')

linetypes = {'-',':','-.','-'};
fs_c = 120;
%%
Plot_Script_JS_multi_oneplot(Cam,EstStruct,linetypes,options)
Plot_Script_JS_error_multi_oneplot(Cam,EstStruct,linetypes,options)
%reproj_error(Cam, EstStruct, options, linetypes)