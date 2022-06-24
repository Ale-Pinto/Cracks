function [xy,p]=d2projectivetrafo(XY,p,dir,FileOut)

% D2PROJECTIVETRAFO performs a projective 2D-transformation with 8 parameters in either transformation 
%                   direction
%
% [xy,p]=d2projectivetrafo(XY,p,dir,FileOut)
%
% Inputs:   XY  nx2-matrix to be transformed. 2xn-matrices are allowed, be careful with
%               2x2-matrices!
%               XY may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%            p  The vector of transformation parameters [a1 a2 a3 a4 a5 a6 a7 a8] where
%
%                    a1*x+a2*y+a3        a4*x+a5*y+a6
%               x' = ------------   y' = ------------
%                    a7*x+a8*y+1         a7*x+a8*y+1
%
%               p may also be a string with the name of a predefined transformation stored in
%               Transformations.mat.
%
%          dir  the transformation direction.
%               If dir=0 (default if omitted or set to []), p are used as given.
%               If dir=1, inverted p' is used to calculate the back-transformation (i.e. if p was
%                  calculated in the direction Sys1 -> Sys2, p' is for Sys2 -> Sys1).
%
%      FileOut  File to write the output to. If omitted, no output file is generated.
%
% Output:   xy  nx2-matrix with the transformed coordinates.
%
%            p  The transformation parameter set which is used. Of course this is only interesting
%               when dir=1 as it returns the inverted input parameters p'.
%
% Systems need to be right-handed, i.e. [x y].
% Parameters may be determined using the helmertprojective2d function.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Dec 20, 2011

%% Do input checking , set defaults and adjust vectors

if nargin<4
    FileOut=[]; 
end
if nargin<3 || isempty(dir)
    dir=0;
elseif ~isscalar(dir)
    error('Parameter ''dir'' must be a scalar expression.')
end
if nargin<2
    error('Too few parameters for d2affinetrafo execution. Check your inputs!')
end
if ischar(p)    
    load Transformations;
    if ~exist(p,'var')
        error(['Transformation set ',p,' is not defined in Transformations.mat - check your definitions!.'])
    elseif (eval(['length(',p,')'])~=8)
        error(['Transformation set ',p,' is of wrong size - check your definitions!.'])
    end
    eval(['p=',p,';']);
end
if numel(p)~=8
    error('Parameter ''p'' must be a 1x8-vector!')
else
    p=p(:)';
end

% Load input file if specified
if ischar(XY)
    XY=load(XY);
end

if (size(XY,1)~=2)&&(size(XY,2)~=2)
    error('Coordinate list XY must be a nx2-matrix!')
elseif (size(XY,1)==2)&&(size(XY,2)~=2)
    XY=XY';
end

%% Do the calculations

if dir  % Calculate backwards parameter set
   p=[p(5)-p(6)*p(8) p(3)*p(8)-p(2) p(2)*p(6)-p(3)*p(5) ...
      -p(4)+p(6)*p(7) -p(3)*p(7)+p(1) -p(1)*p(6)+p(3)*p(4) ...
      p(4)*p(8)-p(5)*p(7) p(2)*p(7)-p(1)*p(8)]/(p(1)*p(5)-p(2)*p(4));
end
xy=zeros(size(XY));
for i=1:size(XY,1)
    xy(i,:)=[p(1)*XY(i,1)+p(2)*XY(i,2)+p(3) p(4)*XY(i,1)+p(5)*XY(i,2)+p(6)]/(p(7)*XY(i,1)+p(8)*XY(i,2)+1);
end

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%12.6f  %12.6f\n',xy');
    fclose(fid);
end