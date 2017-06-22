function Kij=kernelQ_DeVito(xi,xj,beta)
Kij=exp(-beta*(norm(xi-xj))^2);
