##steps
- calculate time distribution in proportion to distance
- Minimun Snap QP Slover
    input: path, ts, n_seg, n_order
    output: coef
    1. compute Q of p'Qp: calculate Q_k of the k-th segment
    2. compute Aeq and bed
        2.1 write expression of Aeq_start and beq_start
        2.2 write expression of Aeq_end and beq_end
        2.3 write expression of Aeq_wp and beq_wp
        2.4 write expression of Aeq_con_p and beq_con_p
        2.5 write expression of Aeq_con_v and beq_con_v
        2.6 write expression of Aeq_con_a and beq_con_a
- display the trajectory