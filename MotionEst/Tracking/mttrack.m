function [y,x]=mttrack(G,y,x,yp0,xp0)

[h,w]=size(G{1});

gx=[-1 0 1];
gy=gx';
u(2,1)=1;
window = 24;
sigma = window/.5;
sz = window*2+1;    % length of gaussFilter vector
x_vec = linspace(-sz / 2, sz / 2, sz);
gaussFilter = exp(-x_vec .^ 2 / (2 * sigma ^ 2));
gaussFilter = gaussFilter / sum (gaussFilter); % normalize
weights = diag(gaussFilter);
%weights = fspecial('gaussian',2*window+1,window/4);
pt=length(y);

for j=1 %number of frames to compute optical flow between
    imx=imfilter(G{j},gx,'symmetric');
    imy=imfilter(G{j},gy,'symmetric');

    imxx=imx.^2;
    imyy=imy.^2;
    imxy=imx.*imy;
    figure
%     subplot(2,1,1); imagesc(G{j});
    imagesc(G{j+1});
for i=1:pt %at which points
    if ~isnan(x(i,j)) && ~isnan(y(i,j))
        u(1,1)=10;
        u(2,1)=10;
        %If the point is on the edge of the image and the window flows over
        %the edge of the image, 
        if x(i,j)<window+1 || x(i,j)>w-window || y(i,j)<window+1 || y(i,j)>h-window 
            y(i,j+1)=NaN;%y(i,j);
            x(i,j+1)=NaN;%x(i,j);
        else
            
            
            xp=xp0(i,j);
            yp=yp0(i,j);

            hold on; plot(xp,yp,'+g')
            
            [Xq1,Yq1]=meshgrid(x(i,j)-window:1:x(i,j)+window,y(i,j)-window:1:y(i,j)+window);
            Vq1=interp2(G{j},Xq1,Yq1);
            Vqxx=interp2(imxx,Xq1,Yq1);
            Vqyy=interp2(imyy,Xq1,Yq1);
            Vqxy=interp2(imxy,Xq1,Yq1);
            Vqx=interp2(imx,Xq1,Yq1);
            Vqy=interp2(imy,Xq1,Yq1);

            VqXX=sum(sum(weights*Vqxx));
            VqYY=sum(sum(weights*Vqyy));
            VqXY=sum(sum(weights*Vqxy));
            cnt = 0;
            while u'*u> 0.1^2 && cnt<1000%(abs(u(1))>.05 || abs(u(2))>.05) && cnt<1000
                cnt = cnt +1;
                [Xq2,Yq2]=meshgrid(xp-window:1:xp+window,yp-window:1:yp+window);
                Vq2=interp2(G{j+1},Xq2,Yq2);

                It  = Vq2-Vq1;
                IXT = sum(sum(weights*Vqx.*It));
                IYT = sum(sum(weights*Vqy.*It));

                u  = -[VqXX VqXY;VqXY VqYY]\[IXT;IYT];
                xp = xp+u(1);
                yp = yp+u(2);
                
                hold on; plot(xp,yp,'.m')
%               subplot(2,1,2); hold on; plot(xp,yp,'.m')
                %pause
                if xp<window+1 || xp>w-window || yp<window+1 || yp>h-window
                    break
                end

            end
            y(i,j+1)=yp;
            x(i,j+1)=xp;
        end
    else
        y(i,j+1) = NaN;
        x(i,j+1) = NaN;
    end

end
end


end

