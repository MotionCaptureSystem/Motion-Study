function [G, L] = pyramidsGL(im, N)
filter = fspecial('Gaussian', 3*6+1, 3);
G{1} = im;
for i = 2:N
    G{i} = imfilter(G{i-1}, filter, 'symmetric');
    G{i} = imresize(G{i}, 0.5, 'nearest');
    L{i-1} = G{i-1} - imfilter(imresize(G{i}, size(G{i-1}), 'nearest'), filter, 'symmetric');
end
L{N} = G{N};
end

