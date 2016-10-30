function [y,x]=mttrack(G,y,x,yp0,xp0)

[h,w]=size(G{1});

gx=[-1 0 1];
gy=gx';
u(2)=1;
window = 15;
pt=length(y);
%%
for j=1 %number of frames to compute optical flow between
for i=1:pt %at which points
    u(1)=1;
    u(2)=1;
    if x(i,j)<window+1 || x(i,j)>w-window || y(i,j)<window+1 || y(i,j)>h-window 
        y(i,j+1)=y(i,j);
        x(i,j+1)=x(i,j);
    else
        imx=imfilter(G{j},gx,'symmetric');
        imy=imfilter(G{j},gy,'symmetric');

        imxx=imx.^2;
        imyy=imy.^2;
        imxy=imx.*imy;

        xp=xp0(i,j);
        yp=yp0(i,j);

        [Xq1,Yq1]=meshgrid(x(i,j)-window:1:x(i,j)+window,y(i,j)-window:1:y(i,j)+window);
        Vq1=interp2(G{j},Xq1,Yq1);
        Vqxx=interp2(imxx,Xq1,Yq1);
        Vqyy=interp2(imyy,Xq1,Yq1);
        Vqxy=interp2(imxy,Xq1,Yq1);
        Vqx=interp2(imx,Xq1,Yq1);
        Vqy=interp2(imy,Xq1,Yq1);

        VqXX=sum(sum(Vqxx));
        VqYY=sum(sum(Vqyy));
        VqXY=sum(sum(Vqxy));
        cnt = 0;
        while (abs(u(1))>0.01 || abs(u(2))>0.01) && cnt<1000
            cnt = cnt +1;
            [Xq2,Yq2]=meshgrid(xp-window:1:xp+window,yp-window:1:yp+window);
            Vq2=interp2(G{j+1},Xq2,Yq2);

            It=Vq2-Vq1;
            IXT=sum(sum(Vqx.*It));
            IYT=sum(sum(Vqy.*It));

            u=-[VqXX VqXY;VqXY VqYY]\[IXT;IYT];
            xp=xp+u(1);
            yp=yp+u(2);
            if xp<window+1 || xp>w-window || yp<window+1 || yp>h-window
                break
            end
        end
        y(i,j+1)=yp;
        x(i,j+1)=xp;
    end

end
end


end

