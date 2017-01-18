function camstruct = log_gauss_var_thresh_feats(camstruct,options)

cams = options.cams;
direct   = options.path;

for cc = cams
    fprintf('\nFinding features in camera number: %i',cc)
    CamNo    =  num2str(cc); %input('Please input the camera number you want to process:','s');
    SFrameNo =  camstruct(cc).start_frame; %input('Please input the first frame number you want to process:'); % First frame you want to process
    EFrameNo =  camstruct(cc).end_frame; %input('Please input the last frame number you want to process:');  % Last frame you want to process

    graymin=250;
    graymax=0;

    To=0.5*(graymin+graymax);
    Tk=To;
    Tk1=0;
    Zo=0;Zb=0;
    n=0;
    k=0;


    bg=imread([direct,filesep,'Cam',CamNo,filesep,'bkgnd.png']);% Back ground image
    bg=rgb2gray(bg);
    h0=fspecial('gaussian',30,20); % Gaussian filter configuration
    bg0=imfilter(bg,h0); % Back ground image blurred
    thre = graythresh(bg0);
    bwbg = im2bw(bg0,thre);
    centers = regionprops(bwbg,'Centroid');
    pixels  = regionprops(bwbg,'PixelList');

    % R=importdata('R.mat');
    % if R<0.006
    %     a=10;b=20;c=5;
    % elseif R>0.01
    %     a=20;b=180;c=13;%%%%%%%%%%%%%%%%%%%%%%改成可调
    % else 
    %     a=20;b=50;c=6;
    % end
    %     writerObj=VideoWriter('batflight.avi');
    %     writerObj.FrameRate = 15;
    %     open(writerObj);
    fprintf('\n')
    fprintf('Processing Image ')
    for FrameNo =  SFrameNo : EFrameNo
        fprintf('%i',FrameNo)
        %     cd('cam1048')
        %     bg=imread('0.png');% Back ground image
        %     bg=bg(:,:,1);
    %     cd('bwbat')
        bwbat=camstruct(cc).bwbat{FrameNo-SFrameNo+1};
        se = strel('disk',10);
        bwbat = imdilate(bwbat,se);
        center=regionprops(bwbat,'Centroid');
        [R]=PERCENTAGE(bwbat);
        if R<0.015
            a1=10;b1=60;c=4;
        elseif R>0.025
            a1=20;b1=350;c=13;%%%%%%%%%%%%%%%%%%%%%%改成可调
        else
            a1=20;b1=110;c=6;
        end
        h1=fspecial('log',[a1 a1],0.43);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%随相机和帧变化
        bg1=imfilter(bg,h1,'conv');

    %     h4=fspecial('gaussian',100,50); % Gaussian filter configuration
    %     bg2=imfilter(bg,h4);

        bat=imread([direct,filesep,'Cam',CamNo,filesep,num2str(FrameNo),'.png']); % Bat image
        bat=bat(:,:,1);
        h2=fspecial('log',[a1 a1],0.44);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        bat1=imfilter(bat,h2,'conv');
        bat_nbg = bat1-bg1;
        h3=fspecial('log',[60 60],0.44); 
        %f=imfilter(bat_nbg,h3,'conv');%log enhancement
        f = bat_nbg;
        while 1
            Tk1=Tk;
            for i=1:720
                for j=1:1280
                    if bwbat(i,j)==1
                        %             if Tk~=Tk1
        %                 Tk1=Tk
                        if f(i,j)<=Tk1
        %                     if f(i,j)>5
                                n=n+1;
                                Zo=(double(f(i,j))+Zo);
        %                     end
                        else
        %                     if f(i,j)>10
                            k=k+1;
                            Zb=(double(f(i,j))+Zb);
        %                     end
                        end
                    end
                end
            end
            o=Zo/n;
            b=Zb/k;
        %     Tk=(1*int8(o)+int8(b))/2;
            Tk=0.6*int8(o)+0.4*int8(b);
            if Tk==Tk1
                break;
            end
        end
        % Tk1=(n*Zo+k*Zb)/(n+k);
        a=zeros(720,1280);
        w_box = 1;
        for i=w_box+1:720-w_box
            for j=w_box+1:1280-w_box
                if bwbat(i-w_box,j-w_box)==1 && bwbat(i+w_box,j+w_box)==1
        %         if bwbat(i,j)==1
                    if f(i,j)>Tk
                        a(i,j)=255;
                    else
                        a(i,j)=0;
                    end
                end
            end
        end

        a = imhmin(f,Tk);
        a = a>150;
        h2=fspecial('gaussian',[3 3],10);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        a=imfilter(imhmin(f,Tk).*uint8(a),h2,'conv');

        b=bwareaopen_wjz(a,b1,8);   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        b=bwareaopen(b,c,8);        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Cross off the noises of white blocks whose area is smaller than 5 (18 

        Points=regionprops(b,'Centroid'); % Coordinates of feature points extracted
%         l=length(Points);
%         figure;
%         clf;
%         imshow(f);
%         hold on
% 
%         for t=1:l % Show the feature points on bat's body
%             centroid = Points(t).Centroid;
%             plot(centroid(1),centroid(2),'.r');
%         end
%         hold off
        camstruct(cc).features{FrameNo-SFrameNo+1}.Location = cat(1,Points.Centroid);
        clear('bwbat')
        if FrameNo ~= EFrameNo
            for indx = 1:length(num2str(EFrameNo))
                fprintf('\b')
            end
        end

    end

end
