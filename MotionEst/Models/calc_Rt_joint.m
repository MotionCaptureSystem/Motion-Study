function Rt = calc_Rt_joint(link_nums, link)
% This function generates the additive uncertainty for the motion model in
% joint space.  The output is a [nDof X nDof] covariance matrix where nDof
% is the number of degrees of freedom of the link LL for which Rt is
% calculated.  If LL is a vector of link numbers, all links are included in
% the covarience matric

%units are radians for angle and mm for distance
tot_nDof = sum([link(link_nums).nDof]);
Rt   = zeros(tot_nDof,tot_nDof);
%%%%Future, consider increasing uncertainty inteligently for groups with
%%%%multiple links*******************
nDof = 0;
for ll = link_nums
    
    inds = nDof+1:nDof + link(ll).nDof;
    if ll == 1
        sigmas = [5^2*ones(1,3),(1*pi/180)^2*ones(1,link(ll).nDof-3)];
        Rt(inds,inds) = diag(sigmas);
        
    elseif regexp(link(ll).nnames, '.*Hum.*')
         Rt(inds,inds) = (10*pi/180)^2*eye(link(ll).nDof);
         
    elseif regexp(link(ll).nnames, '.*Rad.*')
         Rt(inds,inds) = (10*pi/180)^2*eye(link(ll).nDof);
         
    elseif regexp(link(ll).nnames, '.*Met.*')
         Rt(inds,inds) = (5*pi/180)^2*eye(link(ll).nDof);

    elseif  regexp(link(ll).nnames, '.*Phal.*')
         Rt(inds,inds) = (5*pi/180)^2*eye(link(ll).nDof);
         
    elseif  regexp(link(ll).nnames, '.*Wrist.')
         Rt(inds,inds) = (2*pi/180)^2*eye(link(ll).nDof);

    else
         Rt(inds,inds) = (10*pi/180)^2*eye(link(ll).nDof);     %assume position uncertainty equivalent to 5 degrees
    end
    nDof = nDof + link(ll).nDof;
end
    

        