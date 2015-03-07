function env = mergePoly(env)
% mergePoly Merges polytopes that represent the environment
%
% Input: 'env' struct (maps labels to unions of polytopes)
% Output: 'env' struct with polytopes merged
%
% :copyright: 2014 by the California Institute of Technology
% :license: BSD 3-Clause, see LICENSE for details

ops.greedy = 0;     % 0 = optimal, 1 = greedy
ops.verbose = 0;
%     ops.trials = 1;     % num trials to improve greedy soln
ops.overlaps = true;

fields = fieldnames(env);
for i = 1:numel(fields)
    env.(fields{i}) = merge(env.(fields{i}), ops);
end
end
