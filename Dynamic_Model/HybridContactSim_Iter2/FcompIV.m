function contactMode = compIV(x)
% Split up state vector into generalized coordinates and velocities
q = x(1:8);
dq = x(9:16);

% Define constraint functions
a = computeFroga(q');

% Identify possible contact modes where constraint function is 0
possibleModes = find(abs(a)<1e-6);

% Define J, set of contact modes
K = {[],[1],[2],[1;2]};   %[]- no contact  [1] - hind leg contact(lambda_y)  [2] - front leg contact(lambda_y)

% Test each contact mode
for k = 1:length(K)
    % Define current contact mode
    J = K{k};
    notMode = setdiff(possibleModes, J);
    
    % Check that current mode is included in possibleModes, otherwise pass
    if all(ismember(J, possibleModes))
        % Compute reset map into mode
        [dq_p, p_hat] = computeResetMap(x,J);
        
        % Compute reset map into possible modes, reformat the impulses
        [dq_p_union, p_hat_union] = computeResetMap(x,possibleModes);
        
        %p_hat_union(possibleModes) = p_hat_union;
        
        % Generate complementarity conditions
        cond_1 = all(-p_hat>=0);
        cond_2 = all(-p_hat_union(notMode)<0);
        
        % If both complementarity conditions are satisfied, switch modes
        if cond_1 && cond_2
            contactMode = J;
        end
    end
end
end