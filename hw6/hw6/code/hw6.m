clc;clear;close all
v_max = 400;
a_max = 400;
color = ['r', 'b', 'm', 'g', 'k', 'c', 'c'];

%% specify the center points of the flight corridor and the region of corridor
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

%% display the trajectory and cooridor
plot(path(:,1), path(:,2), '*r'); hold on;
for j = 0:n_seg-1
    plot_rect([corridor(j+1,1);corridor(j+1,2)], corridor(j+1,3), corridor(j+1,4));hold on;
end
hold on;

x_pos = [];y_pos = [];
idx = 1;

%% #####################################################
% STEP 4: draw bezier curve
for j = 1:n_seg
    for t = 0:0.01:1
        x_pos(idx) = 0.0;
        y_pos(idx) = 0.0;
        for i = 0:n_order
            basis_p = nchoosek(n_order, i) * t^i * (1-t)^(n_order-i);
%             x_pos(idx) = 
%             y_pos(idx) = 
        end
        idx = idx + 1;
    end
end
% scatter(...);
% plot(...);

function poly_coef = MinimumSnapCorridorBezierSolver(axis, waypoints, corridor, ts, n_seg, c_order, d_order, v_max, a_max)   
    start_cond = [waypoints(1), 0, 0];
    end_cond   = [waypoints(end), 0, 0];   
    
    %% #####################################################
    % STEP 1: compute Q of p'Qp
    [Q, M]  = getQM(n_seg, d_order, ts);
    Q_0 = M'*Q*M;
    Q_0 = nearestSPD(Q_0);
    
    %% #####################################################
    % STEP 2: compute Aeq and beq
    [Aeq, beq] = getAbeq(n_seg, c_order, ts, start_cond, end_cond);
    
    %% #####################################################
    % STEP 3: get corridor_range, Aieq and bieq 
    
    % STEP 3.1: get corridor_range of x-axis or y-axis,
    constraint_range = zeros(n_seg, 2*c_order);

    
    % STEP 3.2: get Aieq and bieq
    [Aieq, bieq] = getAbieq(n_seg, c_order, constraint_range, ts);
    
    f = zeros(size(Q_0,1),1);
    poly_coef = quadprog(Q_0, f, Aieq, bieq, Aeq, beq);
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