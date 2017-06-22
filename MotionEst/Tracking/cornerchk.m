function [y,x]=cornerchk(im)

[h,w]=size(im);

gau=fspecial('gaussian',7,1);
im=imfilter(im,gau,'symmetric');

gx=[-1 0 1];
gy=gx';

imx=imfilter(im,gx,'symmetric');
imy=imfilter(im,gy,'symmetric');

imxx=imx.^2;
imyy=imy.^2;
imxy=imx.*imy;

chi=0.10;
har(h,w)=0;

for i=8:h-7
    for j=8:w-7
        IMXX=sum(sum(imxx(i-7:i+7,j-7:j+7)));
        IMYY=sum(sum(imyy(i-7:i+7,j-7:j+7)));
        IMXY=sum(sum(imxy(i-7:i+7,j-7:j+7)));
        har(i,j)=IMXX*IMYY-IMXY^2-chi*(IMXX+IMYY)^2;
    end
end

THRESHOLD=0.001*max(max(har));
N=5;
har=har.*double(har==ordfilt2(har,N^2,true(N)))>THRESHOLD;

[y,x]=find(har);

end

