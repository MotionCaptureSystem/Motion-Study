function dVdq=dempot(q,Qi,K)

beta=10;
N = size(Qi,2);
lambda_n=.005;
[V,~]=eig(K);
J=V\K*V;

% Spectral Cut-off Method:
g=V*Spectral_cutoff(J/N,lambda_n)*(inv(V));
kq = exp(-beta*(sum((repmat(q,1,N)-Qi).*(repmat(q,1,N)-Qi))));
A = (repmat(q,1,N)-Qi).*repmat(kq,3,1);  

dVdq=4*(-beta)*A*g'/N*kq';
 
