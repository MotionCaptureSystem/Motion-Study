function gJ=Spectral_cutoff(J,lambda_n)

S=size(J);
gJ=zeros(S(1));
for i=1:S(1)
    
    if J(i,i)<=lambda_n 
        gJ(i,i)=1/lambda_n;
    elseif J(i,i)>lambda_n
        gJ(i,i)=1/J(i,i);
    end
end