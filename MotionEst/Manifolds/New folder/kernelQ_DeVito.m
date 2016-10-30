function Kij=kernelQ_DeVito(xi,xj,beta)
Kij=exp(-beta*(xi-xj)'*(xi-xj));
