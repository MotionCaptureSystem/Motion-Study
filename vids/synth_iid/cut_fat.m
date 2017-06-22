function cut_fat(direct)
%CUT_FAT        -Attempts to reduce the file size of all mat files in the
%directory in DIRECT.

files = dir([direct,filesep,'*.mat']);
for ff = 1:length(files)
    sm_file([direct,filesep,files(ff).name])    
end

function sm_file(fname)
load(fname)
save(fname, '-regexp', '^(?!(fname)$).')

