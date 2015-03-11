function st = STLdisplay(phis)
%DISPLAY displays a set of formulas
% 
% Synopsis: st = display(phis)
% 
% Input:
%  - phis : an array of STL formula
%
% Outputs:
%  - st : the string

for ii = 1:numel(phis)
    st = disp(phis(ii),1);
    fprintf(st);
end

fprintf('\n');

end
