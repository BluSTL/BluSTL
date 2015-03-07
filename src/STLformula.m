classdef STLformula
    %STLformula a class to represent an STL formula
    %  
    
    properties
        id
        st
        interval
        type
        phi
        phi1
        phi2
        phin
        params
        params_interval       
    end
    
    methods
        % just a constructor for now
        function phi = STLformula(varargin)
        
            phi.id = varargin{1};
            phi.st = '';
            phi.interval = [0 inf];
            phi.phi = [];
            phi.phi1 = [];
            phi.phi2 = [];
            phi.phin = [];
            phi.type = '';
            phi.params = struct;
            phi.params_interval = struct;
            varargin = varargin(2:end);

            switch numel(varargin)
                case 0
                    phi.st = 'true';
                    phi.type = 'predicate';
        
                otherwise
                    if(numel(varargin)==1 && ischar(varargin{1}))
                        varargin{1} = regexprep(varargin{1},'eventually', 'ev');
                        varargin{1} = regexprep(varargin{1},'always','alw');
                        varargin{1} = regexprep(varargin{1},'<=', '<');
                        varargin{1} = regexprep(varargin{1},'>=', '>');
                    end     
                    phi = STLparse(phi,varargin{:});
            end
        end

        function st = disp(phi,opt)
            st = STLdisp(phi,opt);
        end
                
        function st = display(phi)
            st = STLdisplay(phi);
        end
        
    end
        
end
    

