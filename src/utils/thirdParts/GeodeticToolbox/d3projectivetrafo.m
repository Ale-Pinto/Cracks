function xy=d3projectivetrafo(XYZ,p,FileOut)

% D3PROJECTIVETRAFO performs a projective transformation with 11 parameters from 3D space to plane
%
% xy=d3projectivetrafo(XYZ,p,FileOut)
%
% Inputs:  XZY  nx3-matrix to be transformed. 3xn-matrices are allowed, be careful with
%               3x3-matrices!
%               XYZ may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%            p  The vector of transformation parameters [a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11] where
%
%                    a1*x+a2*y+a3*z+a4         a5*x+a6*y+a7*z+a8
%               x' = ------------------   y' = ------------------
%                    a9*x+a10*y+a11*z+1        a9*x+a10*y+a11*z+1
%
%               p may also be a string with the name of a predefined transformation stored in
%               Transformations.mat.
%
%      FileOut  File to write the output to. If omitted, no output file is generated.
%
% Output:   xy  nx2-matrix with the transformed coordinates.
%
% Systems need to be right-handed, i.e. [x y z] and [x y]
% Parameters may be determined using the helmertprojective3d function.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Dec 21, 2011

%% Do input checking , set defaults and adjust vectors

if nargin<3
    FileOut=[]; 
end
if nargin<2
    error('Too few parameters for d3affinetrafo execution. Check your inputs!')
end
if ischar(p)    
    load Transformations;
    if ~exist(p,'var')
        error(['Transformation set ',p,' is not defined in Transformations.mat - check your definitions!.'])
    elseif (eval(['length(',p,')'])~=11)
        error(['Transformation set ',p,' is of wrong size - check your definitions!.'])
    end
    eval(['p=',p,';']);
end
if numel(p)~=11
    error('Parameter ''p'' must be a 1x11-vector!')
else
    p=p(:)';
end

% Load input file if specified
if ischar(XYZ)
    XY=load(XYZ);
end

if (size(XYZ,1)~=3)&&(size(XYZ,2)~=3)
    error('Coordinate list XYZ must be a nx2-matrix!')
elseif (size(XYZ,1)==3)&&(size(XYZ,2)~=3)
    XYZ=XYZ';
end

%% Do the calculations

xy=zeros(size(XYZ,1),2);
for i=1:size(XYZ,1)
    xy(i,:)=[p(1)*XYZ(i,1)+p(2)*XYZ(i,2)+p(3)*XYZ(i,3)+p(4) p(5)*XYZ(i,1)+p(6)*XYZ(i,2)+p(7)*XYZ(i,3)+p(8)]...
        /(p(9)*XYZ(i,1)+p(10)*XYZ(i,2)+p(11)*XYZ(i,3)+1);
end

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%12.6f  %12.6f\n',xy');
    fclose(fid);
end