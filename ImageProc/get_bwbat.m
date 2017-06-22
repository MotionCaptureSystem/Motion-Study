function camstruct = get_bwbat(camstruct,options)
direct   = options.path;
cams = options.cams;


for cc = cams
    fprintf('\nComputing BW masks for camera %i ...\n',cc)
    CamNo    = num2str(cc); %input('Please input the camera number you want to process:','s');
    SFrameNo =  camstruct(cc).start_frame; %input('Please input the first frame number you want to process:'); % First frame you want to process
    EFrameNo =  camstruct(cc).end_frame; %input('Please input the last frame number you want to process:');  % Last frame you want to process

    if ~exist([direct,filesep,'Cam',CamNo,filesep,'bkgnd.png'],'file')
        vidobj = VideoReader([direct,filesep,'Cam',CamNo,filesep,'cam',CamNo,'.MP4']);
        bg = readFrame(vidobj);
        imwrite(bg,[direct,filesep,'Cam',CamNo,filesep,'bkgnd.png'],'png')
        clear vidobj
    else
        bg=imread([direct,filesep,'Cam',CamNo,filesep,'bkgnd.png']);% Back ground image
    end

    IMG1 = double(bg); 
    IMG2 = IMG1.^1.1;
    indx = IMG2>255;
    IMG2(indx) = 255;

    bg = uint8(IMG2); 
    bg = rgb2gray(bg);

    h1=fspecial('gaussian',20,20); % Gaussian filter configuration
    bg1=imfilter(bg,h1); % Back ground image blurred

    h11=fspecial('gaussian',30,30);
    bg0=imfilter(bg,h11);
    bwbg0 = im2bw(bg0,0.8);
    bwbg0 = ~bwbg0;

    centers=regionprops(bwbg0,'Centroid');
    pixels=regionprops(bwbg0,'PixelList');
    fprintf('Image Number:')
    for FrameNo =  SFrameNo : EFrameNo
        fprintf('%i',FrameNo)
        %%Image Pre-processing
        % The image pre-processing part processes the images from original
        % RGB*3 images into *black and white* images contains only the Feature
        % points.

        bat=imread([direct,filesep,'Cam',CamNo,filesep,num2str(FrameNo),'.png']); % Bat image
        IMG1 = double(bat);
        IMG2 = IMG1.^1.1;
        indx = IMG2>255;
        IMG2(indx) = 255;

        bat = rgb2gray(IMG1);
        bat = uint8(IMG2);

        h2=fspecial('gaussian',20,20);
        bat1=imfilter(bat,h2); % Bat image blurred

        bat_nbg = bg1-bat1;


        threshold = graythresh(bat_nbg);
        % 228 330 0.2
        % 301 0.06
        if strcmp(CamNo,'228')||strcmp(CamNo,'330')
            th = 0.2;
        else
            th = 0.06;
        end

        bwbat{FrameNo-SFrameNo+1}=im2bw(bat_nbg,th);
        center=regionprops(bwbat{FrameNo-SFrameNo+1},'Centroid');
        pixel=regionprops(bwbat{FrameNo-SFrameNo+1}, 'PixelList');
            if length(center)>1
                a=length(pixel(1).PixelList);
                I=1;
                for i=1:length(center)
                    a=length(pixel(i).PixelList);
                    if a<length(pixel(i).PixelList)
                        a=length(pixel(i).PixelList);
                        I=i;
                    end
                end
            else
                I=1;
            end
            n=1;
    %         for m=1:length(center)
            for i=1:length(centers)
    %             dis(i)=sqrt((center(I).Centroid(1,1)-centers(i).Centroid(1,1)).^2+(center(I).Centroid(1,2)-centers(i).Centroid(1,2)).^2);
                m=1;
                %if centers(i).Centroid(1,1)>center(I).Centroid(1,1)-50&&centers(i).Centroid(1,1)<center(I).Centroid(1,1)+50&&centers(i).Centroid(1,2)>center(I).Centroid(1,2)-50&&centers(i).Centroid(1,2)<center(I).Centroid(1,2)+50
                    for j=1:length(pixels(i))
                        if bwbat{FrameNo-SFrameNo+1}(pixels(i).PixelList(j,2),pixels(i).PixelList(j,1))==1
                            overlap.CamNo(n)=i;
                            overlap.Point{n}(m,:)=[pixels(i).PixelList(j,2),pixels(i).PixelList(j,1)];
                            m=m+1;
    %             elseif dis(i)>=40&&dis(i)<150
    %                 t(n)=i;
    %                 n=n+1;
                        end
                    end
                    n=n+1;
               % end
    %             clear('overlap');
            end
    %         end

            if exist('overlap')
                for p=1:length(overlap.CamNo)
                if overlap.CamNo(p)~=0
                    for q=1:length(pixels(overlap.CamNo(p)).PixelList)
                        bwbat{FrameNo-SFrameNo+1}(pixels(overlap.CamNo(p)).PixelList(q,2),pixels(overlap.CamNo(p)).PixelList(q,1))=1;
                    end
                end
                end
            end
            clear('overlap');
            if FrameNo<EFrameNo
                for del_indx = 1:length(num2str(FrameNo))
                fprintf('\b')
                end
            end
    end
    fprintf('\nFilling Holes in Frame:')
    [R]=PERCENTAGE(bwbat{1});
    if R<0.02 
        a=7;
    elseif R>0.035
        a=9;
    else
        a=8;
    end
    % save('R','R');
    for FrameNo =  SFrameNo : EFrameNo
        fprintf('%i',FrameNo)
        p=regionprops(bwbat{FrameNo-SFrameNo+1},'PixelList');
        len=0;
        for l=1:length(p)
            len=length(p(l).PixelList)+len;
        end
        r0=len/(1280*720);
        if r0>=R&&r0<0.02+R
            se1=strel('disk',a);
            bwbat0=imerode(bwbat{FrameNo-SFrameNo+1},se1);
        else
            se1=strel('disk',7);
            %     bwbat=imopen(bwbat,se1);
            bwbat0=imerode(bwbat{FrameNo-SFrameNo+1},se1);
        end
        bwbat3=~bwbat0;
        pixels=regionprops(bwbat3,'PixelList');
        for i=2:length(pixels)
            for j=1:length(pixels(i,1).PixelList(:,1))
                bwbat3(pixels(i,1).PixelList(j,2),pixels(i,1).PixelList(j,1))=0;
            end
        end 
        bwbat0=~bwbat3;
    %     bwbat0=bwbat{FrameNo-SFrameNo+1};
        figure(1);
        imshow(bwbat0);
        pause(0.01)
        %Name=[direct,filesep,'Cam', CamNo,filesep,'bwbat_',num2str(FrameNo)];
        %cd(['Cam' CamNo])
        %save(Name,'bwbat0');
        camstruct(cc).bwbat{FrameNo-SFrameNo+1} = bwbat0;
        clear('bwbat0')
        if FrameNo<EFrameNo
            for del_indx = 1:length(num2str(FrameNo))
                fprintf('\b')
            end
        end
    end
end
