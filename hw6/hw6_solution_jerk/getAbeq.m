function [Aeq, beq] = getAbeq(n_seg, c_order, ts, start_cond, end_cond)
    n_order = 7;
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % Boundary constraints in start point
    Aeq_start = zeros(c_order, n_all_poly);
    for k = 0:c_order-1
        Aeq_start(k+1, 1:k+1) = ...
            factorial(n_order)/factorial(n_order-k)*...
            YangHTriangle(k+1)*ts(1)^(1-k);
    end
    beq_start = start_cond';
    
    %#####################################################
    % Boundary constraints in end point
    Aeq_end = zeros(c_order, n_all_poly);
    for k = 0:c_order-1
        Aeq_end(k+1, n_all_poly-k:n_all_poly) = ...
            factorial(n_order)/factorial(n_order-k)*...
            YangHTriangle(k+1)*ts(n_seg)^(1-k);
    end
    beq_end = end_cond';
    
    %#####################################################
    % p,v,a,j continuity constrain between each 2 segments
    Aeq_con = zeros((n_seg-1)*c_order, n_all_poly);
    beq_con = zeros((n_seg-1)*c_order, 1);
    for k = 0:c_order-1
        start_idx_1 = k*(n_seg-1);
        for j = 0:n_seg-2
            start_idx_2 = (n_order+1)*(j+1);
            Aeq_con(start_idx_1+j+1, start_idx_2-k:start_idx_2) = ...
                      factorial(n_order)/factorial(n_order-k)*...
                      YangHTriangle(k+1)*ts(j+1)^(1-k);
            Aeq_con(start_idx_1+j+1, start_idx_2+1:start_idx_2+1+k) = ...
                         -factorial(n_order)/factorial(n_order-k)*...
                         YangHTriangle(k+1)*ts(j+2)^(1-k);
        end
    end
    
    %#####################################################
    % combine all components to form Aeq and beq 
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end