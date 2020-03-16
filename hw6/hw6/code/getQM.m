function [Q, M] = getQM(n_seg, d_order, ts)
    n_order = 7;
    Q = [];
    M = [];
    M_k = getM(n_order);
    for k = 1:n_seg
        %#####################################################
        % STEP 2.1 calculate Q_k of the k-th segment 
        Q_k = [];

        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_k);
    end
end