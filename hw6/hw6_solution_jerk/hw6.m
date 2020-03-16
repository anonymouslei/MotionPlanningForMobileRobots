clc;clear;close all
v_max = 400;
a_max = 400;
color = ['r', 'b', 'm', 'g', 'k', 'c', 'c'];

% path = ginput() * 100.0;
% x_length = 60;
% y_length = 50;

path = [50, 50;
       100, 120;
       180, 150;
       250, 80;
       280, 0];
   
x_length = 100;
y_length = 100;

c_order = 3; %Ensure continuity at c_order-th order
d_order = 4; %minimize control input at d_order-th order
n_order = 7; % 8 control points
n_seg = size(path, 1);

% define corridor
corridor = zeros(n_seg, 4);
for j = 0:n_seg-1
    corridor(j+1, :) = [path(j+1, 1), path(j+1, 2), x_length/2. y_length/2];
end

% define ts
ts = zeros(n_seg, 1);
for j = 0:n_seg-1
    ts(j+1,1) = 1;
end

poly_coef_x = MinimumSnapCorridorBezierSolver(1, path(:, 1), corridor, ts, n_seg, c_order, d_order, v_max, a_max);
poly_coef_y = MinimumSnapCorridorBezierSolver(2, path(:, 2), corridor, ts, n_seg, c_order, d_order, v_max, a_max);
poly_v_x = [];
poly_v_y = [];
id_v = 1;
for j = 0:n_seg-1
    for i = 0:n_order
        start_idx = j*(n_order+1)+i;
        if i < n_order
            poly_v_x(id_v,:) = (n_order+1) * (poly_coef_x(start_idx+2)-poly_coef_x(start_idx+1));
            poly_v_y(id_v,:) = (n_order+1) * (poly_coef_y(start_idx+2)-poly_coef_y(start_idx+1));
            id_v = id_v+1;    
        end
    end
end
% display the trajectory and cooridor
figure(1)
plot(path(:,1), path(:,2), '*r'); hold on;
for j = 0:n_seg-1
    plot_rect([corridor(j+1,1);corridor(j+1,2)], corridor(j+1,3), corridor(j+1,4));hold on;
end
hold on;

x_pos = [];y_pos = [];
x_vel = [];y_vel = [];
x_acc = [];y_acc = [];
x_jerk = [];y_jerk = [];
idx = 1;
for j = 1:n_seg
    start_pos_id = idx;
    for t = 0:0.01:1
        x_pos(idx) = 0.0;y_pos(idx) = 0.0;
        x_vel(idx) = 0.0;y_vel(idx) = 0.0;
        x_acc(idx) = 0.0;y_acc(idx) = 0.0;
        x_jerk(idx) = 0.0;y_jerk(idx) = 0.0;
        for i = 0:n_order
            start_idx = (j-1)*(n_order+1)+i;
            basis_p = nchoosek(n_order, i) * t^i * (1-t)^(n_order-i);
            x_pos(idx) = x_pos(idx) + poly_coef_x(start_idx+1)*basis_p*ts(j);
            y_pos(idx) = y_pos(idx) + poly_coef_y(start_idx+1)*basis_p*ts(j);
            if i < n_order
                basis_v = nchoosek(n_order-1, i) * t^i *(1-t)^(n_order-1-i);
                x_vel(idx) = x_vel(idx) + (n_order+1) * (poly_coef_x(start_idx+2)-poly_coef_x(start_idx+1))*basis_v;
                y_vel(idx) = y_vel(idx) + (n_order+1) * (poly_coef_y(start_idx+2)-poly_coef_y(start_idx+1))*basis_v;
            end
            if i < n_order-1
                basis_a = nchoosek(n_order-2, i) * t^i *(1-t)^(n_order-2-i);
                x_acc(idx) = x_acc(idx) + (n_order+1) * n_order * (poly_coef_x(start_idx+3) - 2*poly_coef_x(start_idx+2) + poly_coef_x(start_idx+1))*basis_a/ts(j);
                y_acc(idx) = y_acc(idx) + (n_order+1) * n_order * (poly_coef_y(start_idx+3) - 2*poly_coef_y(start_idx+2) + poly_coef_y(start_idx+1))*basis_a/ts(j);
            end
            if i < n_order-2
                basis_j = nchoosek(n_order-3, i) * t^i *(1-t)^(n_order-3-i);
                x_jerk(idx) = x_jerk(idx) + (n_order+1) * n_order * (n_order-1) * ...
                    (poly_coef_x(start_idx+4) - 3*poly_coef_x(start_idx+3) + 3*poly_coef_x(start_idx+2) - poly_coef_x(start_idx+1)) * basis_j/ts(j)^2;
                y_jerk(idx) = y_jerk(idx) + (n_order+1) * n_order * (n_order-1) * ...
                    (poly_coef_y(start_idx+4) - 3*poly_coef_y(start_idx+3) + 3*poly_coef_y(start_idx+2) - poly_coef_y(start_idx+1)) * basis_j/ts(j)^2;
            end
        end
        idx = idx + 1;
    end
    end_pos_id = idx - 1;
    scatter(ts(j)*poly_coef_x((j-1)*(n_order+1)+1:(j-1)*(n_order+1)+1+n_order), ts(j)*poly_coef_y((j-1)*(n_order+1)+1:(j-1)*(n_order+1)+1+n_order), 'filled',color(mod(j,7)+1),'LineWidth', 5);hold on;
    plot(x_pos(start_pos_id:end_pos_id), y_pos(start_pos_id:end_pos_id), color(mod(j,7)+1));hold on;
end

figure(2)
subplot(4,2,1)
plot(x_pos, 'Color', 'r');title('x position');
subplot(4,2,2)
plot(y_pos, 'Color', 'g');title('y position');
subplot(4,2,3)
plot(x_vel, 'Color', 'r');title('x velocity');
subplot(4,2,4)
plot(y_vel, 'Color', 'g');title('y velocity');
subplot(4,2,5)
plot(x_acc, 'Color', 'r');title('x acceleration');
subplot(4,2,6)
plot(y_acc, 'Color', 'g');title('y acceleration');
subplot(4,2,7)
plot(x_jerk, 'Color', 'r');title('x jerk');
subplot(4,2,8)
plot(y_jerk, 'Color', 'g');title('y jerk');

function poly_coef = MinimumSnapCorridorBezierSolver(axis, waypoints, corridor, ts, n_seg, c_order, d_order, v_max, a_max)   
    % c_order = 3, ensure p,v,a continue
    start_cond = [waypoints(1), 0, 0];
    end_cond   = [waypoints(end), 0, 0];  
    %#####################################################
    % STEP 1: compute Q of p'Qp
    [Q, M]  = getQM(n_seg, d_order, ts);
    Q_0 = M'*Q*M;
    Q_0 = nearestSPD(Q_0);
    %#####################################################
    % STEP 2: compute Aeq and beq
    [Aeq, beq] = getAbeq(n_seg, c_order, ts, start_cond, end_cond);
    %#####################################################
    % STEP 3: compute Aieq and bieq 
    constraint_range = zeros(n_seg, 2*c_order);
    for j = 0:n_seg-1
        constraint_range(j+1,:) = [corridor(j+1, axis)+corridor(j+1, 2+axis), -(corridor(j+1, axis)-corridor(j+1, 2+axis)), v_max, v_max, a_max, a_max];
    end
    [Aieq, bieq] = getAbieq(n_seg, c_order, constraint_range, ts);
    f = zeros(size(Q_0,1),1);
    poly_coef = quadprog(Q_0,f,Aieq, bieq, Aeq, beq);
end

function plot_rect(center, x_r, y_r)
    p1 = center+[-x_r;-y_r];
    p2 = center+[-x_r;y_r];
    p3 = center+[x_r;y_r];
    p4 = center+[x_r;-y_r];
    plot_line(p1,p2);
    plot_line(p2,p3);
    plot_line(p3,p4);
    plot_line(p4,p1);
end

function plot_line(p1,p2)
    a = [p1(:),p2(:)];    
    plot(a(1,:),a(2,:),'b');
end