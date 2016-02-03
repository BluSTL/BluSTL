%% Some section title
% 
% <latex>\begin{frame}[fragile]{Some Slide Title}</latex>

%% 
% Some paragraph  
%
% * Some item 1 
% * Some item 2  
%  
% Some text with some formula: $\int f(x) dx$.

%%
% <latex>\bigskip</latex>

%%
% Some matlab code and the output 
fuel_inj_tol = 1.0
MAF_sensor_tol = 1.0

%% 
% <latex>\end{frame}</latex>

%%
% <latex>\begin{frame}[fragile]{Some Code with Figure}</latex>

%% 
% Note that to change the way figure are displayed (size, etc), best is to
% tweak the style file $\mathtt{matlab2latex-pretty.xsl}$.
x= 0:.01:10;
plot(x, cos(x));

%% 
% <latex>\end{frame}</latex>

%% Some subsection
%
% <latex>\begin{frame}[fragile]{Some Last Words}</latex>

%% 
% To compile the latex file, simply run pdflatex main.tex 

%%
% <latex>\bigskip</latex>

%%
% You can add more scripts in the same latex file by adding the
% corresponding input command. 

%%
% <latex>\medskip</latex>
 
%% 
% Note that by default the style file creates an outline slide for each
% section. Also, apparently, you can only have one section per script. 
%

%%
% <latex>\bigskip</latex>

%% 
% If you don't like it, again, simply modify the style file
% $\mathtt{matlab2latex-pretty.xsl}$. Or, add more inline latex, such as 
%

% <latex>\section{Some Other Section Title}</latex>



%% 
% <latex>\end{frame}</latex>




