%% Plot runs

kindef = {'noWrist',...
          'flexBB_both_wings',...
          '2dofPhal',...
          '2dofPhal_noWrist',...
          '2dofPhal_noWrist2',...
          '1dofPhal'};
      
      
for ee = 1:length(kindef)
    load(kindef{ee},'X_ukf','options')
    a(ee).X_ukf = X_ukf;
    options_all(ee) = options;
    clear options X_ukf
    
end

NodeNames   = {'BB_1'       , 1;
               'BB_2'       , 2;
               'BB_3'       , 3;
               'RHum'       , 4;
               'RRad'       , 5;
               'LHum'       , 6;
               'LRad'       , 7;
               'RWrist'     , 8;
               'RD3Met'     , 9;
               'RD4Met'     , 10;
               'RD5Met'     , 11;
               'LWrist'     , 12;
               'LD3Met'     , 13;
               'LD4Met'     , 14;
               'LD5Met'     , 15;
               'LD3Phal1'   , 16;%%%%%%%%%%%
               'LD3Phal2'   , 17;
               'LD4Phal1'   , 18;
               'LD4Phal2'   , 19;
               'LD5Phal1'   , 20;
               'LD5Phal2'   , 21;
               'RD3Phal1'   , 22;%%%%%%%%%%%
               'RD3Phal2'   , 23;
               'RD4Phal1'   , 24;
               'RD4Phal2'   , 25;
               'RD5Phal1'   , 26;
               'RD5Phal2'   , 27};
%%

for ee = 1:length(kindef)
    fs_c = options_all(ee).fs_c;
    link = options_all(ee).link;
    t = 1/fs_c*linspace(0,size(a(ee).X_ukf,2)-1, size(a(ee).X_ukf,2));
    dof_prev = 0;
    dof_int = [];
    cnt = 1;
    
    for ll = 1:length(options_all(ee).link)
    figure(find(strcmp(NodeNames(:,1),options_all(ee).link(ll).nnames)))
    subplot(3,2,ee);
    hold on
        for dof = 1:options_all(ee).link(ll).nDof
            plot(t,180/pi*a(ee).X_ukf(dof+dof_prev,:),options_all(ee).plot.linespec1{dof});
            
        end
        dof_prev = dof_prev+dof;
        title(['Sim: ',kindef{ee}, ' Link: ', options_all(ee).link(ll).nnames])
        if ee==5 ||ee==6
            xlabel('time (s)')
        end
        if ee == 1 || ee == 3 || ee == 5
            ylabel('\theta (deg)')
        end
    end
    
end 
    
    