function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    row = 8*n_seg; 
    col = 4*(n_seg+1);
    Ct = zeros(row,col);
    % fixed derivatives
    % first point correspond
    for i = 1:4
        Ct(i,i) = 1;
    end
    % end point correspond
    for i = 1:4 
        Ct(row-4+i,4+i) = 1;
    end
    % correspond the position at t=0 of the middle point
    for i = 1:n_seg-1
        Ct(i*8+1, 8+i) = 1;
        % previous position at t = T equals next position at t = 0
        Ct(i*8-3, 8+i) = 1;
    end

    % free derivatives
    % correspond the v,a,j at t=0 of the middle point
    for i = 1:n_seg-1
        % v, there are 4+4+n_segment numbers in front of free derivatives 
        Ct(i*8+2, 8+n_seg+3*i) = 1;
        % previous v at t = T equals next velocity at t = 0
        Ct(i*8-2, 8+n_seg+3*i) = 1;
        % a
        Ct(i*8+3, 8+n_seg+3*i+1) = 1;
        % previous a at t = T equals next acceleration at t = 0
        Ct(i*8-2, 8+n_seg+3*i+1) = 1;
        % j
        Ct(i*8+4, 8+n_seg+3*i+2) = 1;
        % previous j at t = T equals next jerk at t = 0
        Ct(i*8-2, 8+n_seg+3*i+2) = 1;
   end

end
