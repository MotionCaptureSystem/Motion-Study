function Vq=empot(q,Qi)
% q is the position that the empirical potential function is calculated at
% Qi is the set of data that act as centers for the kernel function
% or the set of data concenterated around the manifold of interest

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


[V,D]=eig(K);
J=V\K*V;
% size(theta,2)


% Spectral Cut-off Method:
 g=V*Spectral_cutoff(J/N,lambda_n)*(inv(V));

% Tikhonov Regularization Method:
% g=V*Tikhonov_reg(J/size(theta,2),lambda_n)*(inv(V));


kq=zeros(N,1);
for ii=1:N
kq(ii)=kernelQ_DeVito(Qi(ii,:),q,beta);
end        

Vq=1/N*kq'*g*kq;



