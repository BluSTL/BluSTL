function QS = getSpecs(QS)

%% Create the environment
ex = 6;
L = QS.L;

env = envLTL(ex);

% POLYTOPE CONSTRAINTS
% Control constraints
Hu = [eye(3); -eye(3)];
Ku = 0.04*[9.935; 3.62; 3.62;  
           4.545; 3.62; 3.62];
      
% State constraints
p = [0, 10]; % m
v = Inf*[-1, 1]; % m/s
r = Inf*[-1, 1]; % rad
w = Inf*[-1; 1]; % rad/s
Hx = [eye(10); -eye(10)];
Kx = [ p(2)*ones(3, 1);  v(2)*ones(3, 1);  r(2)*ones(2, 1);  w(2)*ones(2, 1);
      -p(1)*ones(3, 1); -v(1)*ones(3, 1); -r(1)*ones(2, 1); -w(1)*ones(2, 1)];

  
fPoly = {strcat('alw (',mat2str(Hx),'*X(:,t) <= ',mat2str(Kx),')'), ...
         strcat('alw (',mat2str(Hu),'*U(:,t) <= ',mat2str(Ku),')')};


G = env.work.safe;
fSafe = {};
if ~isempty(G)
    % Create safety constraints -- disjunction of polytopes
    nPoly = length(G);
    fSafe = strcat('alw (');
    for i = 1:nPoly
            [h,k] = double(G(i));
            if i~=1
                fSafe = [fSafe, ' or ('];
            end
            fSafe = [fSafe, strcat(mat2str(h),'*Y(:,t) <= ',mat2str(k))];
    end
    for j = 1:nPoly
        fSafe = [fSafe, ')'];
    end
else
    fSafe = {};
end


if ex==3
    F =  {env.AP.a, env.AP.b, env.AP.c};
elseif ex==6
	F = {env.AP.a, env.AP.b};
else
	F =  {env.AP.a, env.AP.b, env.AP.c, env.AP.d, env.AP.e};
end


% For patrolling the goal regions in ANY ORDER
if ~isempty(F)
    % Create "goal" constraints
    fGoal = {};
    
    for i = 1:length(F)
        nPoly = length(F{i});   
        
        fGoal{i} = strcat('alw (ev_[0,',int2str(L),'] (');
        for j = 1:nPoly
            [h,k] = double(F{i}(j));
            
            if j~=1
                fGoal{i} = [fGoal{i}, ' or ('];
            end
            fGoal{i} = [fGoal{i}, strcat(mat2str(h),'*Y(:,t) <= ',mat2str(k))];
           
        end
        for j = 1:nPoly-1
            fGoal{i} = [fGoal{i}, ')'];
        end
        fGoal{i} = [fGoal{i}, '))'];
    end
else
    fGoal = {};
end


% % For CYCLIC patrolling of regions
%
% if ~isempty(F)
%     % Create "goal" constraints
%     fGoal = {};
%     
%     nPoly = length(F{end});
%     prevGoal = {};
%     for j = 1:nPoly
%         [h,k] = double(F{end}(j));
% 
%         if j~=1
%             prevGoal = [prevGoal, ' or ('];
%         end
%         prevGoal = [prevGoal, strcat(mat2str(h),'*Y(:,t) <= ',mat2str(k))];
%            
%     end
%     for j = 1:nPoly-1
%         prevGoal = [prevGoal, ')'];
%     end
%     
%     for i = 1:length(F)
%         nPoly = length(F{i});   
%         
%         fGoal{i} = strcat('alw ((');
%         
%         fGoal{i} = [fGoal{i}, strcat(prevGoal{:}, ')','=> (ev_[0,',int2str(L),'] (')];
%         
%         prevGoal = {};
%         for j = 1:nPoly
%             [h,k] = double(F{i}(j));
%             
%             if j~=1
%                 prevGoal = [prevGoal, ' or ('];
%             end
%             prevGoal = [prevGoal, strcat(mat2str(h),'*Y(:,t) <= ',mat2str(k))];
%         end
%         fGoal{i} = [fGoal{i}, prevGoal{:}];
%         for j = 1:nPoly-1
%             fGoal{i} = [fGoal{i}, ')'];
%         end
%         fGoal{i} = [fGoal{i}, ')))'];
%     end
% else
%     fGoal = {};
% end
% %fGoal={};


%fPoly = {};
%fSafe = {};
%fGoal={};


%% STL formula
QS.stl_list = [QS.stl_list, fPoly, fSafe, fGoal{:}];

end

