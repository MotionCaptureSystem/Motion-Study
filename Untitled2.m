for ee = 1:length(kindef)
    fs_c = options_all(ee).fs_c;
    link = options_all(ee).link;
    t = 1/fs_c*linspace(0,size(a(ee).X_ukf,2)-1, size(a(ee).X_ukf,2));
    dof_prev = 0;
    dof_int = [];
    cnt = 1;
    for ll = get_group_links(options_all(ee).link,options_all(ee).groups)
        if ll>1 
            if ~strcmp(options_all(ee).link(ll).nnames,NodeNames{cnt,1})
                cnt = cnt+1;
            end
        end
        
        nDof = link(ll).nDof;
        tDof = link(ll).tDof;
        legend_handles = [];
        
        figure(cnt)
        hold on
        for dof = 1:nDof
            if dof>1
            if tDof(dof) ~= tDof(dof-1)
                cnt = cnt+1;
                h = gca;
                set(h,options_all(ee).plot.fig_txt_props{:})
                textobj = findobj(h, 'type', 'text');
                lineobj = findobj(h, 'type', 'line');
                set(lineobj, 'LineWidth', 2);
                set(textobj,  options_all(ee).plot.fig_txt_props{:});
                handle = legend(legend_handles, options_all(ee).dof_names{dof_prev+1:dof_prev+nDof}, 'Location','NorthEastOutside');
                set(handle,  options_all(ee).plot.fig_txt_props{:});
                legend_handles = [];
                dof_int = dof_prev+dof-1;
            end
            end
            if tDof(dof)
                hold on
                legend_handles(length(legend_handles)+1) = plot(t,a(ee).X_ukf(dof+dof_prev,:),options_all(ee).plot.linespec1{dof});
                clear title xlabel ylabel
                title([options_all(ee).link(ll).nnames,' Displacements',],options_all(ee).plot.fig_txt_props{:})
                xlabel('time (s)',options_all(ee).plot.fig_txt_props{:})
                ylabel('Displacement',options_all(ee).plot.fig_txt_props{:})
                axis tight
            else
                clear title xlabel ylabel
                hold on
                legend_handles(length(legend_handles)+1) = plot(t,180/pi*a(ee).X_ukf(dof+dof_prev,:),options_all(ee).plot.linespec1{dof});
                title([options_all(ee).link(ll).nnames,' Rotations'],options_all(ee).plot.fig_txt_props{:})
                xlabel('time (s)',options_all(ee).plot.fig_txt_props{:})
                ylabel('Angle (deg)',options_all(ee).plot.fig_txt_props{:})
                axis tight
            end
        end
        h = gca;
        set(h,options_all(ee).plot.fig_txt_props{:})
        textobj = findobj(h, 'type', 'text');
        lineobj = findobj(h, 'type', 'line');
        set(lineobj, 'LineWidth', 2);
        set(textobj,  options_all(ee).plot.fig_txt_props{:});
        if ~isempty(dof_int)
            handle = legend(legend_handles, options_all(ee).dof_names{dof_int+1:dof_prev+nDof}, 'Location','NorthEastOutside');
            set(handle,  options_all(ee).plot.fig_txt_props{:});
            dof_int = [];
        else
             handle = legend(legend_handles, options_all(ee).dof_names{dof_prev+1:dof_prev+nDof}, 'Location','NorthEastOutside');
            set(handle,  options_all(ee).plot.fig_txt_props{:});
        end

        dof_prev = dof_prev+nDof;
        %fig_id = gcf;
        %Do the figures need to be saved?
        if options_all(ee).plot.savefig
            print('-r600', '-dpdf', strcat(options_all(ee).plot.savepath,filesep,'JointCoord_',options_all(ee).link_names{ll},'.pdf'))
            %imwrite(fig_id,strcat(options_all(ee).plot.savepath,filesep,'JointCoord_',options_all(ee).link_names{ll},'.png'),'png')
        end
    end
end
   