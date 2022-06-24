function z=d1trafo(Z,p,dir,FileOut)

% D1TRAFO performs 1D-transformation with 2 parameters in either transformation direction
%
% z = d2trafo(Z,p,dir,FileOut)
%
% Inputs:    Z  nx1-matrix to be transformed. 1xn-matrices are allowed.
%               Z may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%            p  The vector of transformation parameters [dz s] with
%                 dz = translations [unit of Z]
%                  s = scale factor
%               p may also be a string with the name of a predefined transformation stored in
%               Transformations.mat. In this case, the following input of O is ignored.
%
%          dir  the transformation direction.
%               If dir=0 (default if omitted or set to []), p are used as given.
%               If dir=1, inverted p' is used to calculate the back-transformation (i.e. if p was
%                  calculated in the direction Sys1 -> Sys2, p' is for Sys2 -> Sys1).
%
%      FileOut  File to write the output to. If omitted, no output file is generated.
%
% Output:   z   nx1-matrix with the transformed coordinates.
%
% Used for transforming cartesian coordinates from one system to another, e.g. when adding relative
% height measurements given by a levelling into cadastral height system.

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
    error('Too few parameters for D1trafo execution. Check your inputs!')
end
if ischar(p)    
    load Transformations;
    if ~exist(p,'var')
        error(['Transformation set ',p,' is not defined in Transformations.mat - check your definitions!.'])
    elseif (eval(['length(',p,')'])~=2)
        error(['Transformation set ',p,' is of wrong size - check your definitions!.'])
    end
    eval(['p=',p,';']);
end
if numel(p)~=2
    error('Parameter ''p'' must be a 1x2-vector!')
else
    p=p(:);
end

% Load input file if specified
if ischar(Z)
    Z=load(Z);
end

if (size(Z,1)~=1)&&(size(Z,2)~=1)
    error('Coordinate list Z must be a nx1-matrix!')
elseif (size(Z,1)==1)&&(size(Z,2)~=1)
    Z=Z';
end

%% Do the calculations

n=size(Z,1);
% Perform transformation
if ~dir
    z=repmat(p(1),n,1)+p(2)*Z;
else
    z=1/p(2)*(Z-repmat(p(1),n,1));
end

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%12.6f\n',z);
    fclose(fid);
end