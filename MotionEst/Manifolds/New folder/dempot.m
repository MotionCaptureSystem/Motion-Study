function dVdq=dempot(q,Qi,K)
beta=10;
lambda_n=.01;

N=size(Qi,2);
%K=zeros(N);
%Create the kernel matrix K -----------------------------------------------
% for ii=1:N
%     for jj=1:N
%         K(ii,jj) = kernelQ_DeVito(Qi(:,ii),Qi(:,jj),beta);
%     end
% end

lambda_n=.005;
[V,D]=eig(K);
J=V\K*V;

% Spectral Cut-off Method:
g=V*Spectral_cutoff(J/N,lambda_n)*(inv(V)); 
kq = exp(-beta*sum((repmat(q,1,N)-Qi).*(repmat(q,1,N)-Qi),1));

A = (repmat(q,1,N)-Qi).*repmat(kq,3,1); 
dVdq=4*(-beta)*A*g'/N*kq';
 
