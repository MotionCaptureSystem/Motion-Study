function x = nearest2lines(lines)
%NEAREST2LINES  -Computes the point X which is nearest to the lines
%contained in the matrix LINES.  X is a point in 2D space.  Lines is a [n x
%4] matrix of points and unit vectors which describe the lines.  A row of
%LINES looks like: [x y x_hat y_hat] where X and Y are the point
%coordinates and X_HAT and Y_HAT are the unit vectors.

n_lines = size(lines,1);
%units = lines(:,3:4);
units = ([0,-1;1,0]*lines(:,3:4)')';
pts = lines(:,1:2);

A = zeros(2,2);
B = zeros(2,1);
for ll = 1:n_lines
    A = A + units(ll,:)'*units(ll,:);
    B = B + units(ll,:)'*units(ll,:)*pts(ll,:)';
end

x = A\B;