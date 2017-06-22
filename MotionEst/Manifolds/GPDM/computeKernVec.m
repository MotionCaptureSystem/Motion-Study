function kvec = computeKernVec(Xin, Xref, theta)

kvec = zeros(size(Xref,1),size(Xin,1));
for ii = 1:size(Xin,1)
    for kk = 1:size(Xref,1)
    kvec(kk,ii) = theta(2)*exp(-theta(1)/2*dist2(Xin(ii,:),Xref(kk,:)));
%         if Xin(ii,:) == Xref(kk,:)
%             kvec(kk,ii) = kvec(kk) + 1/theta(3);
%         end
    end
end