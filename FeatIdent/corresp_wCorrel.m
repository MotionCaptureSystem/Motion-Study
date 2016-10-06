function phi_corr = corresp_optflow(phi_hat,phi,phi_km1,im_km1,im_k)
%finds the closest measurement to each guess. 
%phi_hat    -image space estimate of feature location [ncam*npts*2,1]
%phi        -uncorresponded list of potenttial measurements [ncam*npts*2,1]
%phi_km1    -previous measurment location
%im_km1     -image at the previous timestep
%im_k       -current image

%set patch size
w = 5;
h = w;
%determine the number of predicted points
npts = length(phi_hat)/2;
%initialize the vector for saving corresponded points
phi_corr = zeros(length(phi_hat),1);
%initialize the distance vector
dist = zeros(npts,1);
%initialize the correspondence index vector.
numbers = zeros(npts,1);
%initialize correlation vector
r = zeros(npts,1);

for pp = 1:npts
    %compute the difference between the pp^th estimate and all candiates
    delta = repmat(phi_hat(2*(pp-1)+1:2*pp),length(phi)/2,1) - phi;
    %compute the square sum of distances
    delta_sq = sum(reshape(delta.*delta,2,[]));
    %find the minimum distance and its corresponding index
    [dist(pp),numbers(pp)] = min(delta_sq);
    %if the estimate is within the allowable threshold, save the point.  If
    %not, store NaN.
    if dist(pp)<15^2
        if ~isnan(phi_km1(2*(numbers(pp)-1)+1:2*numbers(pp))) & ~isnan(phi(2*(numbers(pp)-1)+1:2*numbers(pp)))
            row_inds_km1 = round(phi_km1(2*numbers(pp)))-h:round(phi_km1(2*numbers(pp)))+h;
            col_inds_km1 = round(phi_km1(2*(numbers(pp)-1)+1))-w:round(phi_km1(2*(numbers(pp)-1)+1))+w;
            
            rows_out_km1 = (row_inds_km1<1)|(row_inds_km1>720);     %which rows are out of bounds
            row_inds_km1(rows_out_km1) = [];                        %cut them out
            cols_out_km1 = (col_inds_km1<1)|(col_inds_km1>1280);    %which cols are out of bounds
            col_inds_km1(cols_out_km1) = [];                        %cut them out
            
            row_inds_k = round(phi(2*numbers(pp)))-h:round(phi(2*numbers(pp)))+h;
            col_inds_k = round(phi(2*(numbers(pp)-1)+1))-w:round(phi(2*(numbers(pp)-1)+1))+w;
            
            rows_out_k = (row_inds_k<1)|(row_inds_k>720);           %which rows are out of bounds
            row_inds_k(rows_out_k) = [];                            %cut them out
            cols_out_k = (col_inds_k<1)|(col_inds_k>1280);          %which cols are out of bounds
            col_inds_k(cols_out_k) = [];                            %cut them out
            
            trim_cols_k = cols_out_km1;                             %copy col inds from other patch
            trim_cols_k(cols_out_k)   = [];                         %cut inds already chopped
            col_inds_k(trim_cols_k) = [];                           %chop same as other patch 
            
            trim_cols_km1 = cols_out_k;                             %copy col inds from other patch
            trim_cols_km1(cols_out_km1)   = [];                     %cut inds already chopped
            col_inds_km1(trim_cols_km1) = [];                         %cut inds from this patch
            
            trim_rows_k = rows_out_km1;                             
            trim_rows_k(rows_out_k)   = [];
            row_inds_k(trim_rows_k) = [];
            
            trim_rows_km1 = rows_out_k;
            trim_rows_km1(rows_out_km1)   = [];
            row_inds_km1(trim_rows_km1) = [];
            
            im_km1_patch = im_km1(row_inds_km1,col_inds_km1);       %get the image patch
            im_k_patch   = im_km1(row_inds_k,col_inds_k);           %get the image patch
            
            r(pp) = corr2(im_k_patch,im_km1_patch);
        else
            r(pp) = 0;
        end
        phi_corr((2*(pp-1)+1:2*pp),1) = phi(2*(numbers(pp)-1)+1:2*numbers(pp));
    else
        phi_corr((2*(pp-1)+1:2*pp),1) = [NaN;NaN];
    end
end
%npts = 3;
u=unique(numbers);
if size(u,1) ~= npts
    %use the association with best feature match.
    n=histc(numbers,u);
    dup_vals = u(n>1);
    for dd = 1:length(dup_vals)
        idx_dup_vals = find(numbers == dup_vals(dd));
        [~,I] = max(r(idx_dup_vals));
        pt_num_keep = idx_dup_vals(I);
        pt_nums_rej = setdiff(idx_dup_vals,pt_num_keep);
        for zz = 1:length(pt_nums_rej)
            phi_idx_clear(2*(zz-1)+1:2*zz) = 2*(pt_nums_rej(zz)-1)+1:2*pt_nums_rej(zz);
        end
        phi_corr(phi_idx_clear) = NaN;
    end
end
        