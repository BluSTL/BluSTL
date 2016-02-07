function poly = getPoly(varargin)
% poly Creates box polytope from upper/lower bounds
%
% Input: 'x' [xmin xmax ymin ymax zmin zmax]
%        'e' expansion/contraction factor
% Output: 'poly' MPT polytope object
%
% :copyright: 2014 by the California Institute of Technology
% :license: BSD 3-Clause, see LICENSE for details

if nargin == 1
    x = varargin{1};
    e = 0;
    
elseif nargin == 2
    x = varargin{1};
    e = varargin{2};
    
else
    error('Invalid number of arguments.');
end


if length(x) == 4
    xmin = x(1) - e;
    xmax = x(2) + e;
    ymin = x(3) - e;
    ymax = x(4) + e;

    poly = polytope([xmin ymin; xmax ymin; xmax ymax; xmin ymax]);

elseif length(x) == 6
    xmin = x(1) - e;
    xmax = x(2) + e;
    ymin = x(3) - e;
    ymax = x(4) + e;
    zmin = x(5) - e;
    zmax = x(6) + e;

    poly = polytope([xmin ymin zmin; xmax ymin zmin; ...
                     xmin ymax zmin; xmin ymin zmax; ...
                     xmax ymax zmin; xmax ymin zmax; ...
                     xmin ymax zmax; xmax ymax zmax]);

else
    error('Invalid dimension of polytope.');
end

end