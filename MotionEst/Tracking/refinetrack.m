function [y,x]=refinetrack(G,y,x,s)
s1=2;

y=s1*y;
x=s1*x;

[h,w]=size(G{1}{s});

gx=[-1 0 1];
gy=gx';
u(2)=1;

pt=length(y);
%%
for j=1:5
for i=1:pt
    u(1)=1;
    u(2)=1;
    if x(i,j)<8 || x(i,j)>w-7 || y(i,j)<8 || y(i,j)>h-7 % For the write-up part. Check whether a point has moved out. If so, remain its value.
        y(i,j+1)=y(i,j);
        x(i,j+1)=x(i,j);
    else
        imx=imfilter(G{j}{s},gx,'symmetric');
        imy=imfilter(G{j}{s},gy,'symmetric');

        imxx=imx.^2;
        imyy=imy.^2;
        imxy=imx.*imy;

        xp=x(i,j);
        yp=y(i,j);

        [Xq1,Yq1]=meshgrid(x(i,j)-7:1:x(i,j)+7,y(i,j)-7:1:y(i,j)+7);
        Vq1=interp2(G{j}{s},Xq1,Yq1);
        Vqxx=interp2(imxx,Xq1,Yq1);
        Vqyy=interp2(imyy,Xq1,Yq1);
        Vqxy=interp2(imxy,Xq1,Yq1);
        Vqx=interp2(imx,Xq1,Yq1);
        Vqy=interp2(imy,Xq1,Yq1);

        VqXX=sum(sum(Vqxx));
        VqYY=sum(sum(Vqyy));
        VqXY=sum(sum(Vqxy));
%%    
        while abs(u(1))>0.001 || abs(u(2))>0.001

            [Xq2,Yq2]=meshgrid(xp-7:1:xp+7,yp-7:1:yp+7);
            Vq2=interp2(G{j}{s},Xq2,Yq2);

            It=Vq2-Vq1;
            IXT=sum(sum(Vqx.*It));
            IYT=sum(sum(Vqy.*It));

            u=-[VqXX VqXY;VqXY VqYY]\[IXT;IYT];
            xp=xp+u(1);
            yp=yp+u(2);
            if xp<8 || xp>w-7 || yp<8 || yp>h-7
                break
            end
        end
        y(i,j)=yp;
        x(i,j)=xp;
    end

end
end


end

