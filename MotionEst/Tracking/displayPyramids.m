function displayPyramids(G, L)
for i=1:3
    figure(1), subplot(2, 3, i)
    imagesc(G{i})
    colormap(bone)
    figure(1), subplot(2, 3, i+3)
    imagesc(L{i})
end
end