function [CSP, Obsr]=Pts_info()

P= Interface();

[d1,d2,d3]=size(P);         %d1 # of points;d2 # of frames; d3 # of cameras;
CSP=zeros(d1,d2);            %Camera See Points(CSP) matrix, 1st column pts#, 
                            %2nd how many cameras see it.  
                            
cam_num= [{'301'}, '302', '303', '310', '312', '318', '320', '325', '333'];
Obsr=cell(d1,d2);

for j=1 : d2;                 %First loop through pts;
    for i=1 : d1;             %Given a specific point, loop through frame #
    n=0; 
        for k= 1 : d3;        %Given both specific point#&frame#, loop cam       
            if P(i,j,k)==1;
                Obsr(i,j)= strcat(Obsr(i,j),' ',cam_num(k),',');
                n=n+1;
            end
        end
    CSP(i,j)= n;       %For a specific pt% time step(represented by frame#)
                       %how many cams see it.                
    end
end                            
                            


end

