function env = getNotAP(env)
% getNotAP Create negation of APs in env
%
% Input: 'env' struct with all APs
% Output: 'env' struct with all notAPs (negated)
%
% :copyright: 2014 by the California Institute of Technology
% :license: BSD 3-Clause, see LICENSE for details

fields = fieldnames(env.AP);
for i = 1:length(fields)
    notAP = regiondiff(env.work.workspace, env.AP.(fields{i}));
    env.notAP.(fields{i}) = notAP;
end
    
end
