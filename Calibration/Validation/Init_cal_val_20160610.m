%clear all
%close all
%% --------------------------Set Options----------------------------------
%Import Options
options.pts             = [1];
options.pt_names        = {'LED'};
options.tstart          = 1;                  %Note: due to sync delay the first 
options.tstop           = 1000;               %Useable timestep will be tstart+1 
options.interp          = 1;                  %1- data Was NOT interpolated, 0- otherwise;
options.cams            = [301:306,309,311:319,322:331,334:337,339:341];
options.plotflag        = 0;
options.path            = 'C:\ShandongData2016\CalVal11_06142016\Calibration_run\Extrinsic\Svoboda';
options.default_dir     = pwd;

%Stereo Options
options.stereo.pts          = options.pts;
options.stereo.cams         = options.cams;
options.stereo.tstart       = 1;
options.stereo.tstop        = 1000;
options.stereo.dt           = 1;
options.stereo.tsteps       = [options.stereo.tstart:options.stereo.dt:options.stereo.tstop];%[1:20:100,232,310:20:400,573,662];%[232,352,440,573,662,740,840,926];%[1:1000];

%Plot Options
options.plot.tstart         = 1;
options.plot.tstop          = (options.tstop - options.tstart)-(options.plot.tstart-1);
options.plot.linestyle1     = {'.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g','+-r','+-b','+-g', '+-m','+-k','+-c','+--r','+--b','+--g'};
options.plot.linestyle2     = {'+-r','+-b','+-g', '+-m','+-k','+-c','+--r','+--b','+--g','o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g'};
options.plot.linestyle3     = {'o-r','o-b','o-g', 'o-m','o-k','o-c','o--r','o--b','o--g','.-r','.-b','.-g', '.-m','.-k','.-c','.--r','.--b','.--g'};
options.plot.colors         = {'r', 'g', 'b', 'c', 'm', 'k'};
%options.plot.savepath       = 'D:\Users\Matt\Documents\VT\Research\Motion_estimation_dev\Papers_and_Presentations\Bender2015scitech\';
options.plot.savefig        = 0;
options.plot.saveim_reproj  = 0;
options.plot.saveim_reproje = 0;
options.plot.fig_txt_props  = {'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'Bold'};

%% Run the Import Utility 
Cam = load_svoboda_cal4(options);
