function env = envPt2Pt(ex)
% envPt2Pt Creates environment (workspace)
%
% Input: 'ex' number representing the desired environment
% Output: 'env' struct mapping labels to unions of polytopes
%
% :copyright: 2014 by the California Institute of Technology
% :license: BSD 3-Clause, see LICENSE for details

env = struct();
env.util = struct();
env.work = struct();
env.AP = struct();
env.notAP = struct();

e = 0.2;    % expansion/contraction factor
env.util.e = e;

if 2 == ex
    xmin = 0;   xmax = 8;
    ymin = 0;   ymax = 8;
    arena = [xmin, xmax, ymin, ymax];
    workspace = getPoly(arena);

    % Create labled regions
    obs = [ getPoly([2,4, 6,7], e), ...
            getPoly([5,7, 1,2], e), ...
            getPoly([3,5, 3,5], e)];
    env.work.unsafe = obs;
    env.work.safe = regiondiff(workspace, obs);    
    env.AP.goal = getPoly([xmax - 1, xmax, ymax - 1, ymax]);

elseif 3 == ex
    xmin = 0;   xmax = 8;
    ymin = 0;   ymax = 8;
    zmin = 0;   zmax = 8;
    arena = [xmin, xmax, ymin, ymax, zmin, zmax];
    workspace = getPoly(arena);

    % Create labled regions
    obs = [ getPoly([2,4, 6,7, 1,2], e), ...
            getPoly([5,7, 1,2, 4,5], e), ...
            getPoly([3,5, 3,5, 1,7], e)];
    env.work.unsafe = obs;
    env.work.safe = regiondiff(workspace, obs);    
    env.AP.goal = getPoly([7,8, 7,8, 7,8]);
    
else
    error('Invalid example number.');
end

env.util.arena = arena;
env.work.workspace = workspace;
env.util.dim = dimension(workspace);

env = getNotAP(env);

fprintf('Reducing polytopes ...\n');
env.work = mergePoly(env.work);
env.AP = mergePoly(env.AP);
env.notAP = mergePoly(env.notAP);

end
