function [K, R, t] = Pmat2KRt(P)
t = -P(1:3,1:3)\P(1:3,4);

[R,K] = qr(flipud(M)')
K = flipud(K');
K = fliplr(K);

R = R';   
R = flipud(R);

%# make diagonal of K positive
T = diag(sign(diag(K)));

R = R * T;
K = T * K; %# (T is its own inverse)