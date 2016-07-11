function [K, R, t] = Pmat2KRt(P)

M = P(1:3,1:3);
t = -M\P(1:3,4);

[R,K] = qr(flipud(M)');
K = flipud(K');
K = fliplr(K);

R = R';   
R = flipud(R);

% %# make diagonal of K positive
T = diag(sign(diag(K)));

K = K * T;
R = T * R; %# (T is its own inverse)