function P=Interface()


dname='C:\Users\xzfan\Documents\GitHub\MotionStudy\PtsInfo_BACKUP\';
file='CamStruct.mat';
fname=fullfile(dname,file);
load(fname);


P=zeros(34,26,9);

cam_num= [301 302 303 310 312 318 320 325 333]; 

[~,cn2]=size(cam_num);

for k = 1:cn2      
    frame_num = 390- Cam(cam_num(k)).start_frame+floor(Cam(cam_num(k)).sync_del*119.8) +1 : 390-Cam(cam_num(k)).start_frame+floor(Cam(cam_num(k)).sync_del*119.8) +1+35;
    [~,fn2]=size(frame_num);% loop cam 
    for j = 1:fn2  %loop through frame #
        [~,~,pts_dim]=size(Cam(cam_num(k)).pts(1,frame_num(j),:));
        for i=1 : pts_dim                                                      %loop through pts; 
            if isnan(Cam(cam_num(k)).pts(1,frame_num(j),i))==0;
                P(i,j,k)=1;
            end
        end
    
    end

end

end

