function Cam = Bwbat(Cam, options)

working = options.working;
datadir = options.path;

CamNo = options.cams;
  % Last frame you want to process
cam = input('Enter the CAMERA number you wish to track a point in: ');

for cc = cam

    SFrameNo=Cam(cc).start_frame;
    EFrameNo=Cam(cc).end_frame;
%     if isempty(SFrameNo)
%         SFrameNo=Cam(cc).start_frame; % First frame you want to process
%     end
%     if isempty(EFrameNo)
%         EFrameNo=Cam(cc).end_frame;
%     end
    cam_str = num2str(cc);
    while length(cam_str)<3
        cam_str = ['0',cam_str];
    end
    
    bg=imread([datadir,filesep,'Cam',cam_str,filesep,'bkgnd.png']);% Back ground image

    bg=bg(:,:,1);
    h1=fspecial('gaussian',20,20); % Gaussian filter configuration
    bg1=imfilter(bg,h1); % Back ground image blurred
    
    h11=fspecial('gaussian',30,30);
    bg0=imfilter(bg,h11);
    bwbg0 = im2bw(bg0,0.7);
    
    for i=1:720
        for j=1:1280
            if bwbg0(i,j)==0
                bwbg0(i,j)=1;
            else
                bwbg0(i,j)=0;
            end
        end
    end
    centers=regionprops(bwbg0,'Centroid');
    pixels=regionprops(bwbg0,'PixelList');
    
    for FrameNo =  SFrameNo : EFrameNo 
        %% Image Pre-processing
        % The image pre-processing part processes the images from original
        % RGB*3 images into *black and white* images contains only the Feature
        % points.
        %cd (['Cam',num2str(cc)])
        strnum = num2str(FrameNo);
        if length(strnum)<3;
            strnum = strnum;
        end
        bat=imread([datadir,filesep,'Cam',cam_str,filesep,strnum,'.png']); % Bat image

        bat=bat(:,:,1);
        
%         h2=fspecial('gaussian',20,20);
        bat1=imfilter(bat,h1); % Bat image blurred
        
        bat_nbg = bg1-bat1;
        
        
        threshold = graythresh(bat_nbg);
%         bwbat{FrameNo-SFrameNo+1}=im2bw(bat_nbg,0.11);
        bwbat{FrameNo-SFrameNo+1}=im2bw(bat_nbg,threshold/3);
        center=regionprops(bwbat{FrameNo-SFrameNo+1},'Centroid');
        pixel=regionprops(bwbat{FrameNo-SFrameNo+1},'PixelList');
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
        
        for i=1:length(centers)
            m=1;
                for j=1:length(pixels(i))
                    if bwbat{FrameNo-SFrameNo+1}(pixels(i).PixelList(j,2),pixels(i).PixelList(j,1))==1
                        overlap.CamNo(n)=i;
                        overlap.Point{n}(m,:)=[pixels(i).PixelList(j,2),pixels(i).PixelList(j,1)];
                        m=m+1;

                    end
                end
                n=n+1;
        end
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
        
    end

    for FrameNo =  SFrameNo : EFrameNo 

        bwbat0=bwbat{FrameNo-SFrameNo+1};
        Cam(cc).bwbat{FrameNo} = bwbat0;
        
        clear('bwbat0')
    end
    figure(1)
    imshow(bwbat{FrameNo-SFrameNo+1});
    fprintf('BW Masks Created.\n')
end

function [R]=PERCENTAGE(bwbat)
p=regionprops(bwbat{1},'PixelList');
len=0;
for l=1:length(p)
    len=length(p(l).PixelList)+len;
end
R=len/(1280*720);