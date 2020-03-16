function [Aieq, bieq] = getAbieq(n_seg, c_order, constraint_range, ts)
    n_order = 7;
    n_all_poly = n_seg*2*c_order;
    %#####################################################
    % STEP 3.2.1 p constraint
    Aieq_p = [];
    bieq_p = [];

    %#####################################################
    % STEP 3.2.2 v constraint   
    Aieq_v = [];
    bieq_v = [];

    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = [];
    bieq_a = [];
    
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
    Aieq = Aieq_p;
    bieq = bieq_p;
end