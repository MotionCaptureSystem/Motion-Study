function PlotScript(camstruct, eststruct, options)
tstart      = options.plot.tstart;
tstop       = options.plot.tstop;
time        = linspace(tstart,tstop,tstop-tstart+1);
pts         = options.plot.pts;
%pt_names    = options.pt_names;
npts        = length(pts);
cams        = options.cams;
ncam        = length(cams);
nsteps      = tstop-tstart+1;
linestyle1  = options.plot.linespec1;
linestyle2  = options.plot.linespec2;
plot_options = options.plot.fig_txt_props;

features = eststruct.ukf.Features;
pts = options.plot.pts;

nsteps = size(features,2);
figure
hold on
cnt = 0;
for pp = 1:npts
    cnt = cnt+1;
    feat_manip = features(3*(pp-1)+1:3*pp,:);
    plot3(feat_manip(1,:)',feat_manip(2,:)', feat_manip(3,:)', '-.','Color',options.plot.colors(cnt,:))
end

H = zeros(4,4,ncam);
for cc = 1:ncam
    H(:,:,cc) = camstruct(cams(cc)).H;
end
xlabel('x (mm)', 'FontSize', 16)
ylabel('y (mm)', 'FontSize', 16)
zlabel('z (mm)', 'FontSize', 16)
title ('Marker Trajectory in 3D Coordinates', 'FontSize', 16)

h = gca;
set(h,options.plot.fig_txt_props{:})
textobj = findobj(h, 'type', 'text');
lineobj = findobj(h, 'type', 'line');
set(lineobj, 'LineWidth', 1.5);
set(textobj,  options.plot.fig_txt_props{:});

CFPlot(H, .1)
axis tight
axis equal
%set(gca, 'FontSize', 16, 'CameraPosition', [0, 0, 0])





