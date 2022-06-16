function xy=d2trafo(XY,p,dir,FileOut)

% D2TRAFO performs 2D-transformation with 4 parameters in either transformation direction
%
% xy=d2trafo(XY,p,dir,FileOut)
%
% Inputs:   XY  nx2-matrix to be transformed. 2xn-matrices are allowed, be careful with
%               2x2-matrices!
%               XY may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%            p  The vector of transformation parameters [dx dy ez s] with
%                 dx,dy = translations [unit of XY]
%                    ez = rotation in [rad]
%                     s = scale factor
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
% Systems need to be right-handed, i.e. [x y].
% Used for transforming plane cartesian coordinates from one system to another, e.g. when changing 
% from local 2D working system to cadastral mapping system.


% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Dec 09, 2011

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
    error('Too few parameters for D2trafo execution. Check your inputs!')
end
if ischar(p)    
    load Transformations;
    if ~exist(p,'var')
        error(['Transformation set ',p,' is not defined in Transformations.mat - check your definitions!.'])
    elseif (eval(['length(',p,')'])~=4)
        error(['Transformation set ',p,' is of wrong size - check your definitions!.'])
    end
    eval(['p=',p,';']);
end
if numel(p)~=4
    error('Parameter ''p'' must be a 1x4-vector!')
else
    p=p(:);
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

% number of coordinate pairs to transform
n=size(XY,1);

% Create rotation matrix
D=[cos(p(3)) sin(p(3));-sin(p(3)) cos(p(3))];

% Perform transformation
if ~dir
    xy=repmat(p(1:2),1,n)+p(4)*D*XY';
else
    xy=D'/p(4)*(XY'-repmat(p(1:2),1,n));
end
xy=xy';
%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%12.6f  %12.6f\n',xy');
    fclose(fid);
end