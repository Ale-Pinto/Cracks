function CART=ell2cart(ELL,ellips,FileOut)

% ELL2CART performs transformation from ellipsoidal coordinates to cartesian coordinates
% 
% CART=ell2cart(ELL,ellips,FileOut)
% 
% Also necessary:   Ellipsoids.mat   (see beneath)
% 
% Inputs:   ELL  Geographic coordinates as nx3-matrix (longitude, latitude, height) [degree, m]
%                3xn-matrices are allowed. Be careful with 3x3-matrices!
%                nx2-matrices are allowed, all heights are set to 0 in that case.
%                Southern hemisphere is signalled by negative latitude.
%                ELL may also be a file name with ASCII data to be processed. No point IDs, only
%                coordinates as if it was a matrix.
%
%        ellips  The underlying ellipsoid as string in lower case letters, default if omitted or set
%                to [] is 'besseldhdn'
%                See Ellipsoids.m for details.
%                   
%       FileOut  File to write the output to. If omitted, no output file is generated.
%                   
% Outputs: CART  nx3-matrix with right-handed cartesian coordinates (x y z) in [m]

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Jan 18, 2006

%% Do some input checking

% Load input file if specified
if ischar(ELL)
    ELL=load(ELL);
end

% Input size checking and defaults
if ~any(ismember(size(ELL),[2 3]))
    error('Coordinate list ELL must be a nx3- or nx2-matrix!')
elseif (ismember(size(ELL,1),[2 3]))&&(~ismember(size(ELL,2),[2 3]))
    ELL=ELL';
end
if size(ELL,2)==2
    ELL(:,3)=zeros(size(ELL,1),1);
end
if nargin<3
    FileOut=[];
end
if nargin<2 || isempty(ellips)
    ellips='besseldhdn';
end

%% Load ellipsoids
load Ellipsoids;
if ~exist(ellips,'var')
    error(['Ellipsoid ',ellips,' is not defined in Ellipsoids.mat - check your definitions!.'])
end
eval(['ell=',ellips,';']);

%% Do calculations
CART=zeros(size(ELL));
rho=180/pi;
B=ELL(:,2)/rho;
L=ELL(:,1)/rho;

% 1. numerical eccentricity
e2=(ell.a^2-ell.b^2)/ell.a^2;

% norm radius
N=ell.a./sqrt(1-e2*sin(B).^2);

% cartesian coordinates
CART(:,1)=(N+ELL(:,3)).*cos(B).*cos(L);
CART(:,2)=(N+ELL(:,3)).*cos(B).*sin(L);
CART(:,3)=(N.*(1-e2)+ELL(:,3)).*sin(B);

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%12.6f  %12.6f  %12.6f\n',CART');
    fclose(fid);
end