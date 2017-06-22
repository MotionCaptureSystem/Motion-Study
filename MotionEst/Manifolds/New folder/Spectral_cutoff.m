function gJ=Spectral_cutoff(J,lambda_n)

S=size(J);
%gJ=zeros(S(1));
gJ=zeros(1,S(1));
diags_J = diag(J);
inv_diags = diags_J.^-1;

less_lamb = diags_J<=lambda_n;
great_lamb = diags_J>lambda_n;
gJ(less_lamb) = 1/lambda_n;
gJ(great_lamb) = inv_diags(great_lamb);
gJ = diag(gJ);
% for i=1:S(1)
%     if J(i,i)<=lambda_n 
%         gJ(i,i)=1/lambda_n;
%     elseif J(i,i)>lambda_n
%         gJ(i,i)=1/J(i,i);
%     end
% end
