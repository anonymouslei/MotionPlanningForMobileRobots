function M = getM(n_seg, n_order, ts)
    M = [];
    for k = 1:n_seg
        M_k = [];
        M_k = zeros(8,n_order+1);
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        M_k(1,n_order+1) = 1;
        M_k(2,n_order) = 1;
        M_k(3,n_order-1) = 2;
        M_k(4,n_order-2) = 6;
        for i = 1:n_order+1
            M_k(5,i) = ts(k)^(n_order+1-i);
        end
        for i = 1:n_order
            M_k(6,i) = (n_order+1-i)*ts(k)^(n_order-i);
        end
        for i = 1:n_order-1
            M_k(7,i) = (n_order+1-i)*(n_order-i)*ts(k)^(n_order-i-1);
        end
        for i = 1:n_order-2
            M_k(8,i) = (n_order+1-i)*(n_order-i)*(n_order-1-i)*ts(k)^(n_order-i-2);
        end
        M = blkdiag(M, M_k);
    end
end