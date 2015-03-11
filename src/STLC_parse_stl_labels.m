function stl_list = STLC_parse_stl_labels(Sys)
% STLC_parse_stl_labels     Parses the STL specifications of an STLC_lti 
%                           instance and replaces state, output, input and 
%                           disturbance labels with the corresponding indices 
%                           in X, Y, U and W.
%                           
% Input: 
%       Sys: an STLC_lti instance
%
% Output: 
%       stl_list: STL specification over X, Y, U and W.
%
% :copyright: TBD
% :license: TBD

labels = {'xlabel', 'ylabel', 'ulabel', 'wlabel'};
vars = {'X','Y','U','W'};

stl_list = Sys.stl_list;
for istl = 1:numel(Sys.stl_list)
    stl = Sys.stl_list{istl}; 
    for ilabel = 1:numel(labels)
        var = vars{ilabel};
        for jlabel = 1:numel(Sys.(labels{ilabel}))
           label = Sys.(labels{ilabel}){jlabel};
            stl = regexprep(stl,['\<' label '\(t\)'], [ var '(' num2str(jlabel) ',t)'] );             
        end
    end
    stl_list{istl} = stl;
end