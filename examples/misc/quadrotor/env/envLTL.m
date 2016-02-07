function env = envLTL(ex)
%envLTL Creates environment (workspace)
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

e = 0;    % expansion/contraction factor
env.util.e = e;

if 0 == ex
    xmin = 0;   xmax = 8;
    ymin = 0;   ymax = 8;
    arena = [xmin, xmax, ymin, ymax];
    workspace = getPoly(arena);

    % Create labled regions
    obs = [ getPoly([2,4, 5,7], e), ...
            getPoly([4,6, 2,4], e) ];
    env.work.unsafe = obs;
    env.work.safe = regiondiff(workspace, obs);    

    env.AP.a = getPoly([6,7, 5,6]);
    env.AP.b = getPoly([3,4, 0,1]);
    env.AP.c = getPoly([1,2, 3,4]);
    
elseif 2 == ex
    xmin = 0;   xmax = 8;
    ymin = 0;   ymax = 8;
    arena = [xmin, xmax, ymin, ymax];
    workspace = getPoly(arena);

    % Create labled regions
    obs = [ getPoly([2,4, 5,7], e), ...
            getPoly([4,6, 2,4], e) ];
    env.work.unsafe = obs;
    env.work.safe = regiondiff(workspace, obs);    

    env.AP.a = getPoly([6,7, 5,6]);
    env.AP.b = getPoly([3,4, 0,1]);
    env.AP.c = getPoly([1,2, 4,5]);
    
elseif 3 == ex
    xmin = 0;   xmax = 8;
    ymin = 0;   ymax = 8;
    zmin = 0;   zmax = 8;
    arena = [xmin, xmax, ymin, ymax, zmin, zmax];
    workspace = getPoly(arena);

    % Create labled regions
    obs = [ getPoly([2,4, 5,7, 2,6], e), ...
            getPoly([4,6, 2,4, 2,6], e) ];
    env.work.unsafe = obs;
    env.work.safe = regiondiff(workspace, obs);    

    env.AP.a = getPoly([6,7, 6,7, 0,1]);
    env.AP.b = getPoly([3,4, 0,1, 6,7]);
    env.AP.c = getPoly([1,2, 4,5, 4,6]);
    
elseif 4 == ex
    xmin = 0;   xmax = 8;
    ymin = 0;   ymax = 8;
    arena = [xmin, xmax, ymin, ymax];
    workspace = getPoly(arena);

    % Create labled regions
    obs = [ getPoly([2,4, 5,7], e), ...
            getPoly([4,6, 2,4], e), ...
            getPoly([1,2, 7,8], e), ...
            getPoly([1,2, 3,4], e)];
    env.work.unsafe = obs;
    env.work.safe = regiondiff(workspace, obs);    

    env.AP.a = getPoly([6,7, 5,6]);
    env.AP.b = getPoly([3,4, 0,1]);
    env.AP.c = getPoly([1,2, 4,5]);
    env.AP.d = getPoly([7,8, 3,4]);
    env.AP.e = getPoly([4,5, 6,8]);
    
elseif 5 == ex
    xmin = 0;   xmax = 8;
    ymin = 0;   ymax = 8;
    zmin = 0;   zmax = 8;
    arena = [xmin, xmax, ymin, ymax, zmin, zmax];
    workspace = getPoly(arena);

    % Create labled regions
    obs = [ getPoly([2,4, 5,7, 2,6], e), ...
            getPoly([4,6, 2,4, 2,6], e), ...
            getPoly([1,2, 7,8, 2,6], e), ...
            getPoly([1,2, 3,4, 2,6], e)];
    env.work.unsafe = obs;
    env.work.safe = regiondiff(workspace, obs);    

    env.AP.a = getPoly([6,7, 6,7, 0,1]);
    env.AP.b = getPoly([3,4, 0,1, 6,7]);
    env.AP.c = getPoly([1,2, 4,5, 4,6]);
    env.AP.d = getPoly([7,8, 3,4, 4,6]);
    env.AP.e = getPoly([4,5, 6,8, 2,3]);
    
elseif 6 == ex
    xmin = 0;   xmax = 10;
    ymin = 0;   ymax = 10;
    zmin = 0;   zmax = 10;
    arena = [xmin, xmax, ymin, ymax, zmin, zmax];
    workspace = getPoly(arena);

    % Create labled regions
    obs = [ getPoly([2,2.5,2,2.5,0,10], e)];
    env.work.unsafe = obs;
    env.work.safe = regiondiff(workspace, obs);   

    env.AP.a = getPoly([0,1,0,1,0,1]);
    env.AP.b = getPoly([4,5,4,5,4,5]);
    
    %env.AP.a = getPoly([0.5,1.5,0.5,1.5,0,10]);
    %env.AP.b = getPoly([4.5,4.5,0.5,1.5,0,10]);
    
else
    error('Invalid example number.');
end

env.util.arena = arena;
env.work.workspace = workspace;
env.util.dim = dimension(workspace);

env = getNotAP(env);

fprintf('Reducing polytopes...\n');
env.work = mergePoly(env.work);
env.AP = mergePoly(env.AP);
env.notAP = mergePoly(env.notAP);

end
