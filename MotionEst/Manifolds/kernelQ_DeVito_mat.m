function K=kernelQ_DeVito_mat(Qi,beta)
N = size(Qi,2);
K = zeros(N);
for ii = 1:N
    xi = Qi(:,ii);
    K(ii,:)=exp(-beta*sum((repmat(xi,1,N)-Qi).*(repmat(xi,1,N)-Qi)));
end
