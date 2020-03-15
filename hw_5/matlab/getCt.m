function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
        row = 4 * 2 * n_seg;
    col = 4 * (n_seg + 1);
    Ct = zeros(row, col);
    n_fix = 4 + 4 + n_seg - 1;
    
    % start constraint
    Ct(1:4, 1:4) = eye(4);
    idx_fix = 5;
    idx_free = n_fix + 1;
    
    % intermediate constraints
    for i = 1 : n_seg - 1
        start = 4 + 8 * (i - 1);
        for j = start + 1 : start + 4
            if mod(j, 4) == 1
                Ct(j, idx_fix) = 1;
                Ct(j + 4, idx_fix) = 1;
                idx_fix = idx_fix + 1;
            else
                Ct(j, idx_free) = 1;
                Ct(j + 4, idx_free) = 1;
                idx_free = idx_free + 1;
            end
        end
    end
    
    %terminal constraint
    Ct(row - 3 : row , n_fix - 3 : n_fix) = eye(4);
end
