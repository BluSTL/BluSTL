function h = plotEnv(env, obstacles)
% plotEnv Plots labeled environment
%
% Input: 'env' structure representing environment
% Output: Plot of environment
%
% :copyright: 2014 by the California Institute of Technology
% :license: BSD 3-Clause, see LICENSE for details

assert(env.util.dim == 2 || env.util.dim == 3);

h = figure;
hold on

%% Plot the obstacles, not the safe region.
% Obstacle boundaries
ops.shade = 0.1;
ops.color = [0.5, 0.5, 0.5];
plot(obstacles, ops);

% Obstacles
q = env.util.e;
if env.util.dim == 2
    Q = unitbox(2, q);
else
    Q = unitbox(3, q);
end
ops.shade = 0.65;
ops.color = [0.5, 0.5, 0.5];
%plot(minus(obstacles, Q), ops);


%% Plot APs
ops.shade = 0.25;
fields = fieldnames(env.AP);
for i = 1:numel(fields)
    ops.color = 'g';
    plot(env.AP.(fields{i}), ops);
    for j = 1:length(env.AP.(fields{i}))
        pts = extreme(env.AP.(fields{i})(j));
        if env.util.dim == 2
            text(mean(pts(:,1)), mean(pts(:,2)), fields{i}, 'FontSize',18)
        else
            text(mean(pts(:,1)), mean(pts(:,2)), mean(pts(:,3)), fields{i}, 'FontSize',18)
        end
    end
end

% Plot formatting
axis(env.util.arena)
grid off
hXLabel = xlabel('x (m)');
hYLabel = ylabel('y (m)');
set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([hXLabel, hYLabel], ...
    'FontName'   , 'AvantGarde');
set([hXLabel, hYLabel]  , ...
    'FontSize'   , 18          );
set(gca, ...
  'Box'         , 'off'     , ...
  'TickDir'     , 'out'     , ...
  'XColor'      , [.1 .1 .1], ...
  'YColor'      , [.1 .1 .1], ...
  'LineWidth'   , 1         , ...
  'FontSize'    , 16);

grid on
drawnow

end
