function dVdq=dempot(q,Qi)
beta=10;
lambda_n=.01;

N=size(Qi,2);
K=zeros(N);
% Create the kernel matrix K-----------------------------------------------
for i=1:N
    for j=1:N
        K(i,j)=kernelQ_DeVito(Qi(i,:),Qi(j,:),beta);
    end
end

lambda_n=.005;
[V,D]=eig(K);
J=V\K*V;

% Spectral Cut-off Method:
g=V*Spectral_cutoff(J/N,lambda_n)*(inv(V)); 
kq = exp(-beta*sum((repmat(q,1,N)-Qi').*(repmat(q,1,N)-Qi'),1));

A = (repmat(q,1,N)-Qi').*repmat(kq,3,1); 
dVdq=4*(-beta)*A*g'/N*kq';
 
