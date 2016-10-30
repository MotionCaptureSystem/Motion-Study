clc
clear
im(480,512,6)=0;
for i=1:6
    im(:,:,i)=im2single(imread(['./images/hotel.seq' num2str(10*i-10) '.png']));
    [G{i},L{i}]=pyramidsGL(im(:,:,i),3);
end

[y,x]=cornerchk(G{1}{3});
y(:,6)=0;
x(:,6)=0;

[y,x]=mttrack(G,y,x,3);
[y,x]=refinetrack(G,y,x,2);
[y,x]=refinetrack(G,y,x,1);

figure(1),imshow(G{1}{1});
hold on
plot(x(:,1),y(:,1),'.g','Linewidth',3)
hold off
figure(2),imshow(G{1}{1});
hold on
for k=1:size(x)
    plot(x(k,:),y(k,:),'-r','Linewidth',3)
end
hold off
