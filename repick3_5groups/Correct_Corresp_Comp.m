%%Correct Correspondence Comparison
clear all
close all

load Corresp_OptFlow_AllTrack.mat
n_correct_all = n_correct;
clear n_correct

load Corresp_OptFlow_RecTrack.mat
n_correct_rec = n_correct;
clear n_correct

ncam = size(n_correct_all,1);
colors = hsv(ncam);

time = [1:size(n_correct_all,2)-2]';
figure
for cc = 1:ncam
    subplotfill(3,3,cc)
    hold on
    plot(time,n_correct_rec(cc,3:end)','-' ,'color',colors(cc,:),'LineWidth',1.5);
    plot(time,n_correct_all(cc,3:end)',':','color',colors(cc,:),'LineWidth',1.5);
    axis tight
    h(cc) = gca;
    title(['Cam ', num2str(cc)],'FontName','Times New Roman','FontSize',12)
    grid on
    if ismember(cc,[1,4,7])
        ylabel('N Correct','FontName','Times New Roman','FontSize',12)
    end
    if ismember(cc,[7,8,9])
        xlabel('Frame','FontName','Times New Roman','FontSize',12)
    end
    if ismember(cc,[1:6])
        set(gca,'XTickLabel',[]);
    end
    if ~ismember(cc,[1,4,7])
        set(gca,'YTickLabel',[])
    end
end

ymax = 0; 
for cc = 1:ncam
    ylim = get(h(cc),'Ylim');
    if ymax<ylim(2)
        ymax = ylim(2);
    end
end

for cc = 1:ncam
set(h(cc),'Ylim',[0,ymax])
end

figure
hold on
col = {'k','r','b','g','c'};
cnt = 0;
for cc = [6,9]
    cnt = cnt+1;
    plot(time,n_correct_rec(cc,3:end)','-' ,'color',col{cnt},'LineWidth',1.5);
    plot(time,n_correct_all(cc,3:end)','--','color',col{cnt},'LineWidth',1.5);

end
axis tight
set(gca,'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold')

ylabel('Correct Correspondences','FontName','Times New Roman','FontSize',12)
xlabel('Frame Number','FontName','Times New Roman','FontSize',12)

