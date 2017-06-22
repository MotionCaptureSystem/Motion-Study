%%TEST_CORRESP -Test script for correspondence algorithms

%% Create the Image Space Measurements
phi = creat_meas;

for kk = 3:size(phi,2)
    %meas prediction
    phi_hat = 2*phi(kk-1) - phi(kk-2);
    %correspondence
    phi_corr = corresp(phi_hat,phi(:,kk));
    %update
    phi(:,kk) = phi_corr;    
    
end