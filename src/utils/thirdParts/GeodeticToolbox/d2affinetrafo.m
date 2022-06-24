function xy=d2affinetrafo(XY,p,dir,FileOut)

% D2AFFINETRAFO performs an affine 2D-transformation with 6 parameters in either transformation direction
%
% xy=d2affinetrafo(XY,p,dir,FileOut)
%
% Inputs:   XY  nx2-matrix to be transformed. 2xn-matrices are allowed, be careful with
%               2x2-matrices!
%               XY may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%            p  The vector of transformation parameters [x0 y0 a1 a2 a3 a4] with
%                 x0 y0    = translations [unit of XY]
%                 a1 to a4 = linear affine parameters
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
% Parameters may be determined using the helmertaffine2d function.

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
    elseif (eval(['length(',p,')'])~=6)
        error(['Transformation set ',p,' is of wrong size - check your definitions!.'])
    end
    eval(['p=',p,';']);
end
if numel(p)~=6
    error('Parameter ''p'' must be a 1x6-vector!')
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

xy=zeros(size(XY));
for i=1:size(XY,1)
    if ~dir
        xy(i,:)=[p(1:2)+[p(3:4)';p(5:6)']*XY(i,:)']';
    else
        xy(i,:)=inv([p(3:4)';p(5:6)'])*(XY(i,:)-p(1:2)')';
    end
end

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%12.6f  %12.6f\n',xy');
    fclose(fid);
end