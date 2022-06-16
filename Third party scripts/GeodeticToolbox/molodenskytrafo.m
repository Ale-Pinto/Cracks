function ELL2=molodenskytrafo(ELL1,p,ellips,dir,FileOut)

% MOLODENSKYTRAFO performs a Molodensky transformation on ellipsoidal coordinates with 5 parameters
%                 in either transformation direction
%
% ELL2=molodenskytrafo(ELL1,p,ellips,dir,FileOut)
%
% Also usable:   Transformations.mat   (see beneath)
%
% Inputs: ELL1  nx3-matrix to be transformed. 3xn-matrices are allowed, be careful with
%               3x3-matrices!
%               Each line contains a point with [Longitude Latitude Height] in [degree, m]
%               to be transformed.
%               If no height is used, also nx2 matrices are allowed; input dimension 3x2 or 2x3 
%               will always be regarded as height included.
%               ELL1 may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%            p  The vector of transformation parameters [dX dY dZ da df] with
%                 dX,dY,dZ = translations of ellipsoid centers [m]
%                       da = difference of ellipsoid semimajor axes [m]
%                       df = difference of ellipsoid flattening
%
%               p may also be a string with the name of a predefined transformation stored in
%               Transformations.mat.
%
%       ellips  The underlying starting ellipsoid as string in lower case letters, default if 
%               omitted or set to [] is 'besseldhdn'.
%               If dir = 1 (invers transformation), ellips is the target ellipsoid!
%               See Ellipsoids.m for details.
%
%          dir  the transformation direction.
%               If dir=0 (default if omitted or set to []), p are used as given.
%               If dir=1, inverted p' is used to calculate the back-transformation (i.e. if p was
%                  calculated in the direction Sys1 -> Sys2, p' is for Sys2 -> Sys1).
%
%      FileOut  File to write the output to. If omitted, no output file is generated.
%
% Output: ELL2  nx3-matrix with the transformed coordinates.
%
% Used for transforming ellipsoidal coordinates from one system to another, e.g. when changing 
% the reference ellipsoid,  called "datum transformation" in geodesy.
% Might be of poor quality for bigger transformation areas. For precise geodetic datum transformations,
% you need identical points in both systems to determine parameters using helmert3d function.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Munich, October 24th, 2013

%% Do input checking , set defaults and adjust vectors

if nargin<5
    FileOut=[]; 
end
if nargin<4 || isempty(dir)
    dir=0;
elseif ~isscalar(dir)
    error('Parameter ''dir'' must be a scalar expression.')
end

if nargin<3 || isempty(ellips) ,ellips='besseldhdn';end

% Load ellipsoids
load Ellipsoids;
if ~exist(ellips,'var'), error(['Ellipsoid ',ellips,' is not defined in Ellipsoids.mat - check your definitions!.'])
end
eval(['ell=',ellips,';']);

if nargin<2
    error('Too few parameters for Molodenskytrafo execution. Check your inputs!')
end
if ischar(p)    
    load Transformations;
    if ~exist(p,'var')
        error(['Transformation set ',p,' is not defined in Transformations.mat - check your definitions!.'])
    elseif (length(p)~=5)
        error(['Transformation set ',p,' is of wrong size - check your definitions!.'])
    end
    eval(['p=',p,'(1:5);']);
end
if numel(p)~=5
    error('Parameter ''p'' must be a 1x5-vector!')
else
    p=p(:);
end

% Load input file if specified
if ischar(ELL1)
    ELL1=load(ELL1);
end

if (size(ELL1,1)~=3)&&(size(ELL1,2)~=3)
    if (size(ELL1,1)~=2)&&(size(ELL1,2)~=2)
        error('Coordinate list XYZ must be a nx3-matrix (or nx2-matrix)!')
    else
        if (size(ELL1,1)==2)
            Ell1=Ell1';
        end
        Ell1 = [Ell1 zeros(size(Ell1,1),1)];
    end        
elseif (size(ELL1,1)==3)&&(size(ELL1,2)~=3)
    ELL1=ELL1';
end

if (dir~=0) % inverse transformation
   ell.a=ell.a+p(4);
   ell.f=ell.f+p(5);
   p=-p;
end

%% Do the calculations

% number of coordinate triplets to transform
n=size(ELL1,1);
ELL2=zeros(size(ELL1));

% Calculate the correction values:
e2=(ell.a^2-ell.b^2)/ell.a^2;
ELL1(:,1:2)=ELL1(:,1:2).*pi./180;
for i=1:n
   N=ell.a./sqrt(1-e2*sin(ELL1(i,2)).^2);
   M=ell.a.*(1-e2)./(1-e2*sin(ELL1(i,2)).^2).^(1.5);
   dlat=1/(M+ELL1(i,3))*(-p(1)*sin(ELL1(i,2))*cos(ELL1(i,1))-p(2)*sin(ELL1(i,2))*sin(ELL1(i,1))+...
       p(3)*cos(ELL1(i,2))+(N*e2*sin(ELL1(i,2))*cos(ELL1(i,2)))/ell.a*p(4)+...
       sin(ELL1(i,2))*cos(ELL1(i,2))*(M/(1-ell.f)+N*(1-ell.f))*p(5));
   dlong=1/((N+ELL1(i,3))*cos(ELL1(i,2)))*(-p(1)*sin(ELL1(i,1))+p(2)*cos(ELL1(i,1)));
   dh=p(1)*cos(ELL1(i,2))*cos(ELL1(i,1))+p(2)*cos(ELL1(i,2))*sin(ELL1(i,1))+p(3)*sin(ELL1(i,2))-...
       ell.a/N*p(4)+N*(1-ell.f)*sin(ELL1(i,2)).^2*p(5);
   ELL2(i,:)=ELL1(i,:)+[dlong dlat dh];
end
ELL2(:,1:2)=ELL2(:,1:2).*180./pi;

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%12.6f  %12.6f  %12.6f\n',ELL2');
    fclose(fid);
end