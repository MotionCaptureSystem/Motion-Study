function gJ=Tikhonov_reg(J,lambda_n)

S=size(J);

for i=1:S(1)
    gJ(i,i)=1/(lambda_n+J(i,i));
end
