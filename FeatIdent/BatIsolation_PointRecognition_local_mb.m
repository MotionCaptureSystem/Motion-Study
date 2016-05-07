function Cam = BatIsolation_PointRecognition_local_mb(Cam, options)
%% BatIsolation_PointRcognition
% BatIsolation_PointRcognition.m does the pre-processing and the feature
% extraction of frame images of bats flight motion. The script consists of
% three different parts: the initialization, the image pre-processing and
% the feature extraction.
% Funtion: Pre-processing and Feature Extraction
% *Note that* the configuration in image processing is based on the specific
% condition of bat flight motion image data. It is not assumed to be fitted
% to any other image data. User who wants to use this script should
% consider re-config the parameters of pre-processing part and even some
% parameters in Feature Extraction.

%% 	Initialization
% Initialization part does the basic configuration of the script, the path,
% the folder to read for example. User who use the same archive as the
% author's do not have to change this part but it would be necessary to
% modify if the archive is not same.

working = options.working;
datadir = options.path;

CamNo = options.cams;
  % Last frame you want to process
cam = input('Enter the CAMERA number you wish to track a point in: ');

for cc = cam
    SFrameNo=input('Enter the FIRST frame you want to process? Enter = first frame: ');
    EFrameNo=input('Enter the LAST frame you want to process? Enter = last frame: ');
    if isempty(SFrameNo)
        SFrameNo=Cam(cc).start_frame; % First frame you want to process
    end
    if isempty(EFrameNo)
        EFrameNo=Cam(cc).end_frame;
    end
    cam_str = num2str(cc);
    while length(cam_str)<3
        cam_str = ['0',cam_str];
    end
    
    bg=imread([datadir,filesep,'Cam',cam_str,filesep,'bkgnd.png']);% Back ground image
    bg=bg(:,:,1);
    h0=fspecial('gaussian',30,20); % Gaussian filter configuration
    bg0=imfilter(bg,h0); % Back ground image blurred
    thre = graythresh(bg0);
    bwbg=im2bw(bg0,thre);
    centers=regionprops(bwbg,'Centroid');
    pixels=regionprops(bwbg,'PixelList');

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
        %figure;imshow(bat);
        
        %cd ../
        
        
        %         threshold = graythresh(bat_nbg);
        %         bwbat0 = im2bw(bat_nbg,threshold); % Black and white pattern of bat's body
        
        
        
        %%
        % Local adaptive threshold segmentation
        %cd('bwbat')
        
        

        if ~isempty(Cam(cc).bwbat{FrameNo})
            bwbat = Cam(cc).bwbat{FrameNo};
            center=regionprops(bwbat,'Centroid');
            
            
            [R]=PERCENTAGE(bwbat);
            if R<0.012
                a=10;b=30;c=5;
            elseif R>0.02
                a=20;b=200;c=13;
            else
                a=20;b=50;c=6;
            end
            
            h1=fspecial('log',[a a],0.43);
            bg1=imfilter(bg,h1,'conv');
            h2=fspecial('log',[a a],0.44);
            bat1=imfilter(bat,h2,'conv');
            
            bat_nbg = bat1-bg1;
            h3=fspecial('log',[50 50],0.44);
            bat_nbg=imfilter(bat_nbg,h3,'conv');%log enhancement
            thre = graythresh(bat_nbg);
            graybat=im2bw(bat_nbg,thre);
            %         graybat2=graybat;
            
            

            bwbat0=uint8(bwbat).*bat;
            %     thr = graytresh(bwbat0);
            bwbat2=im2bw(bwbat0,0.1);
            bwbat1=bwbat-bwbat2;
            bwbat11=bwbat1;
            bwbat3=~bwbat1;
            bwbat33=bwbat3;
            pixels=regionprops(bwbat3,'PixelList');
            
            for i=2:length(pixels)
                for j=1:length(pixels(i,1).PixelList(:,1))
                    bwbat3(pixels(i,1).PixelList(j,2),pixels(i,1).PixelList(j,1))=0;
                end
            end
            
            bwbat1=~bwbat3;
            
            for i=1:720
                for j=1:1280
                    if bwbat1(i,j)==0
                        graybat(i,j)=0;
                    end
                end
            end

            
            
            BW=bwareaopen_wjz(graybat,b,8);
            
            BW1=bwareaopen(BW,c,8); % Cross off the noises of white blocks whose area is smaller than c (8 neighbor)
            %     imshow(BW1);
            
            Points=regionprops(BW1,'Centroid');
            %         l=length(Points);
            figure(1)
            imshow(bat);
            hold on
            % Show the feature points on bat's body
            centroids = reshape([Points.Centroid],2,[]);
            plot(centroids(1,:)',centroids(2,:)','*r');hold on
            % put the feature points in the camera structure
            Cam(cc).im_feat{FrameNo} = centroids;
        end
        
    end
    
end
fprintf('Done Identifying Features.\n')
end


% FrameNo=input('Please input the first frame number you want to process:'); 
% cd ('FrameFolder')

% bg=imread('Frames_1.jpg');% Back ground image
% bg=bg(:,:,1);
%     imshow(bg);
%     title('Background image')



function [R]=PERCENTAGE(bwbat)
p=regionprops(bwbat,'PixelList');
len=0;
for l=1:length(p)
    len=length(p(l).PixelList)+len;
end
R=len/(1280*720);
end



function bw2 = bwareaopen_wjz(varargin)
%BWAREAOPEN Remove small objects from binary image.
%   BW2 = BWAREAOPEN(BW,P) removes from a binary image all connected
%   components (objects) that have fewer than P pixels, producing another
%   binary image BW2.  This operation is known as an area opening.  The
%   default connectivity is 8 for two dimensions, 26 for three dimensions,
%   and CONNDEF(NDIMS(BW),'maximal') for higher dimensions. 
%
%   BW2 = BWAREAOPEN(BW,P,CONN) specifies the desired connectivity.  CONN
%   may have the following scalar values:  
%
%       4     two-dimensional four-connected neighborhood
%       8     two-dimensional eight-connected neighborhood
%       6     three-dimensional six-connected neighborhood
%       18    three-dimensional 18-connected neighborhood
%       26    three-dimensional 26-connected neighborhood
%
%   Connectivity may be defined in a more general way for any dimension by
%   using for CONN a 3-by-3-by- ... -by-3 matrix of 0s and 1s.  The
%   1-valued elements define neighborhood locations relative to the center
%   element of CONN.  CONN must be symmetric about its center element.
%
%   Class Support
%   -------------
%   BW can be a logical or numeric array of any dimension, and it must be
%   nonsparse.
%
%   BW2 is logical.
%
%   Example
%   -------
%   Remove all objects in the image text.png containing fewer than 50
%   pixels.
%
%       BW = imread('text.png');
%       BW2 = bwareaopen(BW,50);
%       imshow(BW);
%       figure, imshow(BW2)
%
%   See also BWCONNCOMP, CONNDEF, REGIONPROPS.

%   Copyright 1993-2011 The MathWorks, Inc.
%   $Revision: 1.10.4.9 $  $Date: 2011/11/09 16:48:52 $

% Input/output specs
% ------------------
% BW:    N-D real full matrix
%        any numeric class
%        sparse not allowed
%        anything that's not logical is converted first using
%          bw = BW ~= 0
%        Empty ok
%        Inf's ok, treated as 1
%        NaN's ok, treated as 1
%
% P:     double scalar
%        nonnegative integer
%
% CONN:  connectivity
%
% BW2:   logical, same size as BW
%        contains only 0s and 1s.

[bw,p,conn] = parse_inputs(varargin{:});

CC = bwconncomp(bw,conn);
area = cellfun(@numel, CC.PixelIdxList);

idxToKeep = CC.PixelIdxList(area <= p);
idxToKeep = vertcat(idxToKeep{:});

bw2 = false(size(bw));
bw2(idxToKeep) = true;
end

%%%
%%% parse_inputs
%%%
function [bw,p,conn] = parse_inputs(varargin)

narginchk(2,3)

bw = varargin{1};
validateattributes(bw,{'numeric' 'logical'},{'nonsparse'},mfilename,'BW',1);
if ~islogical(bw)
    bw = bw ~= 0;
end

p = varargin{2};
validateattributes(p,{'double'},{'scalar' 'integer' 'nonnegative'},...
    mfilename,'P',2);

if (nargin >= 3)
    conn = varargin{3};
else
    conn = conndef(ndims(bw),'maximal');
end
iptcheckconn(conn,mfilename,'CONN',3)
end







