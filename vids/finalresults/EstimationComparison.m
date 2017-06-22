%% Compare Estimation Techniques
clear all
close all

load('condind.mat');
EstStruct(1) = eststruct;
EstStruct(1).ukf.X = EstStruct(1).ukf.X(:,1:end);
EstStruct(1).kinc = EstStruct(1).kinc(1:end);
load('std.mat','eststruct');
EstStruct(2) = eststruct;
EstStruct(2).ukf.X = EstStruct(2).ukf.X(:,1:end);
EstStruct(2).kinc = EstStruct(2).kinc(1:end);
load('iid.mat','eststruct');
EstStruct(3) = eststruct;
EstStruct(3).ukf.X = EstStruct(3).ukf.X(:,1:end);
EstStruct(3).kinc = EstStruct(3).kinc(1:end);

load('condind.mat','Cam','options')

fprintf('data loaded ...\n')

linetypes = {'-','--','-.',':'};
fs_c = 120;
%%
Plot_Script_JS_multi_oneplot(Cam,EstStruct,linetypes,options)
%reproj_error(Cam, EstStruct, options, linetypes)