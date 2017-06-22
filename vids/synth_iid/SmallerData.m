%% Convert Data Structures to Smaller File Formats

load EstStructStandard.mat
ukf     = EstStruct.ukf;
kinc    = EstStruct.kinc;
options = EstStruct.options;
save('EstStructStandardSM.mat','ukf','kinc','options') 
fprintf('File 1 Saved.\n')

load EstStructRecursive.mat
ukf     = EstStruct.ukf;
kinc    = EstStruct.kinc;
options = EstStruct.options;
save('EstStructRecursiveSM.mat','ukf','kinc','options') 
fprintf('File 2 Saved.\n')

load EstStructStandardWrist.mat
ukf     = EstStruct.ukf;
kinc    = EstStruct.kinc;
options = EstStruct.options;
save('EstStructStandardWristSM.mat','ukf','kinc','options') 
fprintf('File 3 Saved.\n')

load EstStructRecursiveWrist.mat
ukf     = EstStruct.ukf;
kinc    = EstStruct.kinc;options = EstStruct.options;
save('EstStructRecursiveWristSM.mat','ukf','kinc','options') 
fprintf('File 4 Saved.\n')