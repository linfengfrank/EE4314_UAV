function [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond, acc_contraint)
%GETABEQ Construct equality constraint matrices Aeq and beq for minimum snap QP.
%   Inputs:
%       n_seg     - Number of segments
%       n_order   - Polynomial order (e.g. 7)
%       waypoints - Waypoint vector [x1; x2; ...; xn]
%       ts        - Time durations for each segment
%       start_cond - Start boundary condition [pos, vel, acc, jerk]
%       end_cond   - End boundary condition [pos, vel, acc, jerk]
% 
%   Outputs:
%       Aeq, beq  - Linear equality constraint matrices for QP

    n_coef = n_order + 1;
    n_vars = n_seg * n_coef;

    %% Start Constraints (position, velocity, acceleration, jerk)
    Aeq_start = zeros(4, n_vars);
    beq_start = start_cond(:);

    for k = 0:3
        row = derivativeConstraintRow(0, k, n_order);
        Aeq_start(k+1, 1:n_coef) = row;
    end

    %% End Constraints (position, velocity, acceleration, jerk)
    Aeq_end = zeros(4, n_vars);
    beq_end = end_cond(:);

    for k = 0:3
        % [To be completed] Define the end constraints here
        row = ;
        Aeq_end(k+1, (n_seg-1)*n_coef + 1 : n_seg*n_coef) = row;
    end

    %% Waypoint Constraints (position at internal waypoints)
    Aeq_wp = zeros(n_seg - 1, n_vars);
    beq_wp = waypoints(2:end-1);

    for j = 1:n_seg-1
        % [To be completed] Define the waypoint constraints here
        row = ;
        Aeq_wp(j, (j-1)*n_coef + 1 : j*n_coef) = row;
    end

    %% Continuity Constraints Across Segments (p, v, a, j)
    Aeq_con = [];
    for k = 0:3
        Aeq_con_k = continuityConstraint(n_seg, n_order, ts, k);
        Aeq_con = [Aeq_con; Aeq_con_k];
    end
    beq_con = zeros(size(Aeq_con, 1), 1);

    %% Acceleration Constrains
    Aeq_acc = zeros(n_seg - 1, n_vars);
    beq_acc = acc_contraint;

    for j = 1:n_seg-1
        row = derivativeConstraintRow(ts(j), 2, n_order);
        Aeq_acc(j, (j-1)*n_coef + 1 : j*n_coef) = row;
    end

    %% Combine All Constraints
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con; Aeq_acc];
    beq = [beq_start; beq_end; beq_wp; beq_con; beq_acc];
end


function row = derivativeConstraintRow(t, k, n_order)
%Derivative Contstrains: Generates a row for k-th derivative at time t
    row = zeros(1, n_order+1);
    for i = k:n_order
        row(i+1) = factorial(i) / factorial(i-k) * t^(i-k);
    end
end

function Aeq_k = continuityConstraint(n_seg, n_order, ts, k)
%Continous Constrains: Generates k-th derivative continuity constraints
    n_coef = n_order + 1;
    n_vars = n_seg * n_coef;
    Aeq_k = zeros(n_seg-1, n_vars);

    for j = 1:n_seg-1
        t = ts(j);
        row_pre  = derivativeConstraintRow(t, k, n_order);
        row_post = derivativeConstraintRow(0, k, n_order);

        Aeq_k(j, (j-1)*n_coef + 1 : j*n_coef)     = row_pre;
        Aeq_k(j, j*n_coef + 1 : (j+1)*n_coef) = -row_post;
    end
end


