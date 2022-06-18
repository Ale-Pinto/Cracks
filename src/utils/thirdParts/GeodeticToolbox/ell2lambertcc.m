function PRO=ell2lambertcc(ELL,sys,UND,FileOut)

% ELL2lambertcc performs transformation from ellipsoidal coordinates to a conic Lambert mapping projection
%               using two standard parallels
%
% PRO=ell2lambertcc(ELL,sys,UND,FileOut)
%
% Also necessary:   Projections.mat   Ellipsoids.mat   (see beneath)
%
% Inputs:  ELL  Coordinates on ellipsoid as nx2-matrix (longitude, latitude) [degree]
%               2xn-matrices are allowed. Be careful with 2x2-matrices!
%               If for (some) points ellipsoidic height is available, ELL may be a nx3-matrix also.
%               Southern hemisphere is signalled by negative latitude.
%               ELL may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%          sys  The projection system type as string in lower case letters
%               Default if omitted or set to [] is 'bev'.
%               Information about the projection systems is stored in the mat-File "Projections.mat"
%               which has cell-array members named by the projection type, e.g. 'bev' for Lambert 
%               projection with Austrian settings given by the BEV.
%              
%          UND  Undulation values from geoid model to calculate projected height from ellipsoidal height.
%               If omitted or set to [], no correction is done on the height in PRO.
%               Ellipsoidal height = Projected height + Undulation value
%                   
%      FileOut  File to write the output to. If omitted, no output file is generated.
%
% Outputs: PRO  nx2- resp. nx3- matrix with abscissa, ordinate (and height) in the projection system
%
% ell2lambertcc is meant as last step for geodetic transformations, when global coordinates or other
% projections have to be transformed to any conical lambert projection coordinates.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Aug 26, 2011

%% Do some input checking

% Load input file if specified
if ischar(ELL)
    ELL=load(ELL);
end

% Check input size and defaults
if ~any(ismember(size(ELL),[2 3]))
    error('Coordinate list ELL must be a nx2- or nx3-matrix!')
elseif (ismember(size(ELL,1),[2 3]))&&(~ismember(size(ELL,2),[2 3]))
    ELL=ELL';
end
n=size(ELL,1);  % Number of coordinates to transform
if nargin<4
    FileOut=[];
end
if nargin<3 || isempty(UND)
    UND=zeros(n,1);
elseif (numel(UND)~=n)||(~isvector(UND))
    error('Parameter ''UND'' must be a vector with the length of ELL!')
else
    UND=UND(:)';
end
if nargin<2 || isempty(sys)
    sys='bev';
end

%% Load projection types
load Projections;
% Search for right projection type
if ~exist(sys,'var')
    error(['Projection type ''',sys,'''is not defined in Projections.mat - check your definitions!'])
end
% Get projection values
eval(['TYPE=',sys,'.type;']);
if ~strcmp(TYPE,'lambertcc2')
    error(['Projection type ''',sys,''' is not a Lambert CC 2 projection!']);
end
eval(['LAT=',sys,'.lat;']);
eval(['ellips=',sys,'.ellips;']);
eval(['ORell=',sys,'.ORell;']);
eval(['ORproj=',sys,'.ORproj;']);

%% Load ellipsoids
load Ellipsoids;
if ~exist(ellips,'var')
    error(['Ellipsoid ',ellips,' is not defined in Ellipsoids.mat - check your definitions!.'])
end
eval(['ell=',ellips,';']);

%% Do calculations
PRO=zeros(size(ELL));
rho=180/pi;
ELL(:,1:2)=ELL(:,1:2)/rho;
LAT=LAT/rho;
ORell=ORell/rho;

% Calculation constants
e=sqrt((ell.a^2-ell.b^2)/ell.a^2);
t1=tan(pi/4-LAT(1)/2)/((1-e*sin(LAT(1)))/(1+e*sin(LAT(1))))^(e/2);
t2=tan(pi/4-LAT(2)/2)/((1-e*sin(LAT(2)))/(1+e*sin(LAT(2))))^(e/2);
t0=tan(pi/4-ORell(2)/2)/((1-e*sin(ORell(2)))/(1+e*sin(ORell(2))))^(e/2);
m1=cos(LAT(1))/sqrt(1-e^2*sin(LAT(1))^2);
m2=cos(LAT(2))/sqrt(1-e^2*sin(LAT(2))^2);

n=(log(m1)-log(m2))/(log(t1)-log(t2));
F=m1/(n*t1^n);
r0=ell.a*F*t0^n;

% Calculation values of the points to be transformed
t=tan(pi/4-ELL(:,2)/2)./((1-e*sin(ELL(:,2)))./(1+e*sin(ELL(:,2)))).^(e/2);
r=ell.a*F*t.^n;

gamma=n*(ELL(:,1)-ORell(1));

PRO(:,2)=ORproj(2)+r0-r.*cos(gamma);
PRO(:,1)=ORproj(1)+r.*sin(gamma);

% Height:
if (size(ELL,2)==3),
    PRO(:,3)=ELL(:,3)-UND;
end

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    if (size(ELL,2)==3)
        fprintf(fid,'%12.6f  %12.6f  %12.6f\n',PRO');
    else
        fprintf(fid,'%12.6f  %12.6f\n',PRO');
    end
    fclose(fid);
end