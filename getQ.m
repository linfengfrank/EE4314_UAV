function Q = getQ(n_seg, n_order, ts)
%getQ Construct block-diagonal matrix Q for minimum snap cost.
%   Inputs:
%       n_seg   - Number of trajectory segments
%       n_order - Polynomial order (e.g. 7 for 8 coefficients)
%       ts      - Segment durations (n_seg x 1 vector)
%
%   Output:
%       Q       - Block diagonal cost matrix for all segments

    n_coef = n_order + 1;
    Q = zeros(n_seg * n_coef);

    for k = 1:n_seg
        Q_k = zeros(n_coef);  % Q for segment k

        % Compute Q_k for snap cost (4th derivative)
        for i = 4:n_order
            for j = 4:n_order
                
                % [To be completed] Compute Q_k here
                val = ;
                Q_k(i+1, j+1) = val;
            end
        end

        % Place Q_k into block diagonal position of Q
        idx = (k-1)*n_coef + 1 : k*n_coef;
        Q(idx, idx) = Q_k;
    end
end