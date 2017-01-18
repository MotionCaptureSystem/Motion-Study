function Rt = calc_Rt_gpdm(n_bb_dof,n_lat_dof)

sig_bb = 100;
sig_lat = .05;

Rt = [sig_bb*eye(n_bb_dof),zeros(n_bb_dof,n_lat_dof);
      zeros(n_lat_dof,n_bb_dof),sig_lat*eye(n_lat_dof)];