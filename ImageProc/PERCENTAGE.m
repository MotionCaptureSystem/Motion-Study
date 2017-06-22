%
function [R]=PERCENTAGE(bwbat)
p=regionprops(bwbat,'PixelList');
len=0;
for l=1:length(p)
    len=length(p(l).PixelList)+len;
end
R=len/(1280*720);
 

