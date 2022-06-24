function ELL=cart2ell(CART,ellips,FileOut)

% CART2ELL performs transformation from cartesian coordinates to ellipsoidal coordinates 
% 
% ELL=cart2ell(CART,ellips,FileOut)
% 
% Also necessary:   Ellipsoids.mat   (see beneath)
% 
% Inputs:  CART  Right-handed cartesian coordinates as nx3-matrix (X,Y,Z) [m]
%                3xn-matrices are allowed. Be careful with 3x3-matrices!
%                CART may also be a file name with ASCII data to be processed. No point IDs, only
%                coordinates as if it was a matrix.
%
%        ellips  The underlying ellipsoid as string in lower case letters, default if omitted or set
%                to [] is 'besseldhdn'
%                See Ellipsoids.m for details.
%                   
%       FileOut  File to write the output to. If omitted, no output file is generated.
%
% Outputs:  ELL  nx3-matrix with ellipsoidal coordinates (longitude, latitude, ell. height)
%                in [degree, m]
%                Southern hemisphere is signalled by negative latitude.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Jan 18, 2006

%% Do some input checking

% Load input file if specified
if ischar(CART)
    CART=load(CART);
end

% Check input sizes
if     (size(CART,1)~=3)&&(size(CART,2)~=3), error('Coordinate list CART must be a nx3-matrix!')
elseif (size(CART,1)==3)&&(size(CART,2)~=3), CART=CART';
end

% Defaults
if nargin<3, FileOut='';end
if nargin<2 || isempty(ellips) ,ellips='besseldhdn';end

% Load ellipsoids
load Ellipsoids;
if ~exist(ellips,'var'), error(['Ellipsoid ',ellips,' is not defined in Ellipsoids.mat - check your definitions!.'])
end
eval(['ell=',ellips,';']);

%% Do calculations

ELL=zeros(size(CART));
% Longitude
ELL(:,1)=atan2(CART(:,2),CART(:,1))*180/pi;
ELL(ELL(:,1)<0,:)=ELL(ELL(:,1)<0,:)+360;

% Latitude
B0=atan2(CART(:,3),sqrt(CART(:,1).^2+CART(:,2).^2));
B=100*ones(size(B0));
e2=(ell.a^2-ell.b^2)/ell.a^2;
while(any(abs(B-B0)>1e-10))
    N=ell.a./sqrt(1-e2*sin(B0).^2);
    h=sqrt(CART(:,1).^2+CART(:,2).^2)./cos(B0)-N;
    B=B0;
    B0=atan((CART(:,3)./sqrt(CART(:,1).^2+CART(:,2).^2)).*(1-e2*N./(N+h)).^(-1));
end
ELL(:,2)=B*180/pi;
ELL(:,3)=h;

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%12.10f  %12.10f  %12.10f\n',ELL');
    fclose(fid);
end