function [Aieq, bieq] = getAbieq(n_seg, c_order, constraint_range, ts)
    n_order = 7;
    n_all_poly = n_seg*2*c_order;
    Aieq = zeros(2*n_seg*c_order*(n_order+1-(c_order-1)/2), n_all_poly);
    bieq = zeros(2*n_seg*c_order*(n_order+1-(c_order-1)/2), 1);
    
    %#####################################################
    % p,v,a,j 
    for k = 0:c_order-1
        for j = 0:n_seg-1
            start_idx_1 = 2*n_seg*k*(n_order+1-(k-1)/2)+2*j*(1+n_order-k);
            start_idx_2 = (n_order+1)*j;
            for i = 0:n_order-k
                Aieq(start_idx_1+2*i+1:start_idx_1+2*i+2, start_idx_2+i+1:start_idx_2+i+1+k) = ...
                    [YangHTriangle(k+1)*factorial(n_order)/factorial(n_order-k)*ts(j+1)^(1-k);
                    -YangHTriangle(k+1)*factorial(n_order)/factorial(n_order-k)*ts(j+1)^(1-k)];
                bieq(start_idx_1+2*i+1:start_idx_1+2*i+2, 1) = constraint_range(j+1,2*k+1:2*k+2)';
            end
        end
    end
end