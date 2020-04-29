function [T, idx] = sicp_SE2(target, source, initialT)
    % Generalized ICP over SE(3)
    %
    %   Author: Maani Ghaffari Jadidi
    %   Date:   02/26/2020
    %
    %   Modified to add semantic Labeling
    %   Author: Alexander Jaeckel
    %   Date:   04/26/2020

    % Create kd-tree objects for NN queries
    target_xyz = squeeze(double(target.Location));
    source_xyz = squeeze(double(source.Location));
    source_Label = source.Label;
    target_Label = target.Label;
    target_kdt = KDTreeSearcher(target_xyz);
    source_kdt = KDTreeSearcher(source_xyz);

    % Covariance normal at each point 
    Ct = pc_covariances_ann(target_kdt);
    Cs = pc_covariances_ann(source_kdt);

    % Initial guess
%     T0 = eye(4);
%      T0 = [0 -.9 0 0; .9 0 0 0; 0 0 1 0; 0 0 0 1];
    T0 = initialT;
    T1 = T0;

    % ICP loop: find correspondences and optimize
    d_threshold = 10000;
    converged = false;
    tf_epsilon = 1e-10;
    iter = 0;
    max_iter = 100;
    inner_max_iter = 100;
    eps_Jr = 1e-6;
    idx = [];
    while ~converged && iter < max_iter
        % apply the current transformation to the source point cloud
        current_source = source_xyz * T0(1:3,1:3)';
        current_source(:,1) = current_source(:,1) + T0(1,4);
        current_source(:,2) = current_source(:,2) + T0(2,4);
        current_source(:,3) = current_source(:,3) + T0(3,4);
        idx = zeros(size(source_xyz,1),1);
        % NN queries
        for i = 1:size(source_xyz,1)
            current_source_entry = current_source(i,:); 
            curLabel = source_Label(i);
            Label_idx = find(target_Label == curLabel);
            Labeled_target = target_xyz(Label_idx,:);
                              
            %index of Labeled target
            if ~isempty(Labeled_target)
                idx(i) = Label_idx(knnsearch(Labeled_target, current_source_entry));
            end
        end
        % apply distance threshold to remove outliers
        dist = zeros(size(idx,1),1)-1;
        for i = 1:size(idx,1)
            if idx(i) ~= 0
                dist(i) = sqrt(sum((current_source(i,:) - target_kdt.X(idx(i),:)).^2,2));
            end
            %dist = sqrt(sum((current_source - target_kdt.X(idx,:)).^2,2))
        end    
         survived_idx = find(dist < d_threshold & dist >= 0);
         target_idx = idx(dist < d_threshold & dist >= 0);

         p_source = source_xyz(survived_idx,:);
         p_target = target_kdt.X(target_idx,:);
        
        % solve for the new transformation
        % Gauss-Newton solver over SE(3)
        inner_iter = 0;
        while inner_iter < inner_max_iter
            inner_iter = inner_iter + 1;
            % solve normal equations
            [A,b] = compute_jacobian(T1);
            dx = A \ b;
            % retract and update the estimate
            T1 = expm( hat(dx) ) * T1;

            if ~mod(inner_iter,5)
                disp(['GN Iter: '     num2str(inner_iter)])      
            end   

            % check if converged
            if norm(b) < eps_Jr
                if mod(inner_iter,5)
                disp(['GN Iter: '     num2str(inner_iter)])      
                end
                break;
            end
        end

        % check if converged
        if norm(logm(T0 \ T1)) < tf_epsilon
            disp('Converged')
            converged = true;
        else
            T0 = T1;
            iter = iter + 1;
            disp(['Iter: '     num2str(iter)])
            if ~(iter < max_iter)
                disp(['Not converged. Maximum iteration of ', num2str(max_iter), ' is reached'])
            end
        end
    end

    T = T1;

    % A and b for GN
    function [A,b] = compute_jacobian(X)
        A = zeros(6);
        b = zeros(6,1);
        R = X(1:3,1:3);
        t = X(1:3,4);
        % residual
        r = p_target - p_source * R';
        r(:,1) = r(:,1) - t(1);
        r(:,2) = r(:,2) - t(2);
        r(:,3) = r(:,3) - t(3);
        n = size(r,1);
        for i = 1:n
            % Inverse of covariance Cholesky factor
            invL = chol(eye(3) + R * eye(3) * R', 'lower') \ eye(3);
%           invL = eye(3);
            % Jacobian
            J = invL * [skew(p_source(i,:)'), -eye(3)];
            % Left hand side matrix A x = b
            A = A + J' * J;
            % Right hand side vector A x = b
            b = b - J' * invL * r(i,:)';
        end
        
        for i = 1:6
            if A(i,i) == 0
                A(i,i) = .0001;
            end
        end
    end

    function C = pc_covariances_ann(pckdt)
    % Compute the empirical covariance at each point using an ANN search
    % setting this to identity for icp and not gicp
        e = 1e-2; % covariance epsilon
        C = cell(size(pckdt.X,1),1);
        for i = 1:length(C)
            C{i} = eye(3)+.0001*randn;
        end
    end

%     function X = skew(x)
%     % vector to skew R^3 -> so(3)
% %     X = [0 -x(3) 0;
% %         x(3) 0   0
% %         0   0   0];
% 
% X = [   0,  -x(3),  x(2);
%     x(3),      0,  -x(1);
%     -x(2), x(1),   0];
%     end
% 
    function X = hat(x)
    % hat: R^6 -> se(3)
%    X = [skew(x(1:3)), x(4:6); 0 0 0 0];
    temp = [0 -x(3) 0;
        x(3) 0   0
        0   0   0];
    X = [temp x(4:6); 0 0 0 0];
    end

function X = skew(x)
% vector to skew R^3 -> so(3)
X = [   0,  -x(3),  x(2);
    x(3),      0,  -x(1);
    -x(2), x(1),   0];
end

% function X = hat(x)
% % hat: R^6 -> se(3)
% X = [skew(x(1:3)), x(4:6); 0 0 0 0];
% end
end
