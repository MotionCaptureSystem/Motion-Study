function plot_states(Cam, EstStruct, options)

if strcmp(options.est.type,'joint')
    Plot_Script_JS(Cam, EstStruct, options)
elseif strcmp(options.est.type,'point')
    PlotScript(Cam, EstStruct, options)
end