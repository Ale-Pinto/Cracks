function xyz=d3affinetrafo(XYZ,p,dir,FileOut)

% D3AFFINETRAFO performs an affine 3D-transformation with 12 parameters in either transformation direction
%
% xyz=d3affinetrafo(XYZ,p,dir,FileOut)
%
% Also usable:   Transformations.mat   (see beneath)
%
% Inputs:  XYZ  nx3-matrix to be transformed. 3xn-matrices are allowed, be careful with
%               3x3-matrices!
%               XYZ may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%            p  The vector of transformation parameters [x0 y0 z0 a1 a2 a3 a4 a5 a6 a7 a8 a9] with
%                 x0 y0 z0 = translations [unit of XYZ]
%                 a1 to a9 = linear affine parameters
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
% Output:  xyz  nx3-matrix with the transformed coordinates.
%
% Systems need to be right-handed, i.e. [x y z].
% Parameters may be determined using the helmertaffine3d function.

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
    error('Too few parameters for d3affinetrafo execution. Check your inputs!')
end
if ischar(p)    
    load Transformations;
    if ~exist(p,'var')
        error(['Transformation set ',p,' is not defined in Transformations.mat - check your definitions!.'])
    elseif (eval(['length(',p,')'])~=12)
        error(['Transformation set ',p,' is of wrong size - check your definitions!.'])
    end
    eval(['p=',p,';']);
end
if numel(p)~=12
    error('Parameter ''p'' must be a 1x12-vector!')
else
    p=p(:);
end

% Load input file if specified
if ischar(XYZ)
    XYZ=load(XYZ);
end

if (size(XYZ,1)~=3)&&(size(XYZ,2)~=3)
    error('Coordinate list XYZ must be a nx3-matrix!')
elseif (size(XYZ,1)==3)&&(size(XYZ,2)~=3)
    XYZ=XYZ';
end

%% Do the calculations

xyz=zeros(size(XYZ));
for i=1:size(XYZ,1)
    if ~dir
        xyz(i,:)=[p(1:3)+[p(4:6)';p(7:9)';p(10:12)']*XYZ(i,:)']';
    else
        xyz(i,:)=inv([p(4:6)';p(7:9)';p(10:12)'])*(XYZ(i,:)-p(1:3)')';
    end
end

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%12.6f  %12.6f  %12.6f\n',xyz');
    fclose(fid);
end