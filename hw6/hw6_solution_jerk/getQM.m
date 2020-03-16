function [Q, M] = getQM(n_seg, d_order, ts)
    n_order = 2*d_order-1;
    Q = [];
    M = [];
    M_j = getM(n_order);
    for j = 0:n_seg-1
        Q_j = zeros(n_order+1, n_order+1);
        for i = d_order:n_order
            for l = i:n_order
                Q_j(i+1, l+1) = factorial(i)/factorial(i-d_order)*...
                    factorial(l)/factorial(l-d_order)*...
                    ts(j+1)^(3-2*d_order) / (i+l-n_order);
                if i ~= l
                   Q_j(l+1, i+1) = Q_j(i+1, l+1); 
                end
            end
        end
        Q = blkdiag(Q, Q_j);
        M = blkdiag(M, M_j);
    end
end

