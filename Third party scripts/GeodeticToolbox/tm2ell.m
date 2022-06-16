function ELL=tm2ell(PRO,sys,UND,FileOut)

% TM2ELL performs transformation from a transverse mercator mapping projection to ellipsoidal coordinates
%
% ELL=TM2ell(PRO,sys,UND,FileOut)
%
% Also necessary:   Projections.mat   Ellipsoids.mat   (see beneath)
%
% Inputs:  PRO  coordinates in projection system as nx2-matrix (abscissa and ordinate vaules).
%               2xn-matrices are allowed. Be careful with 2x2-matrices!
%               If for (some) points height is available, PRO may be a nx3-matrix also.
%               Input coordinates must be complete, i.e. they need to contain the zone identifier ID
%               at the beginning of easting coordinate.
%               PRO may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%          sys  is the projection system type as string in lower case letters
%               Default if omitted or set to [] is 'gk' (German GK projection)
%               Information about the projection systems is stored in the mat-File "Projections.mat"
%               which has cell-array members named by the projection type, e.g. 'gk' for
%               Gauss-Kruger projection.
%               Feel free to add individual tm projections.
%               See Projections.m for details.
%
%          UND  undulation values from geoid model to calculate projected height from ellipsoidal height.
%               If omitted or set to [], no correction is done on the height in ELL.
%               Ellipsoidal height = Projected height + Undulation value
%                   
%      FileOut  File to write the output to. If omitted, no output file is generated.
%
% Outputs: ELL nx2- resp. nx3- matrix with longitude, latitude (and height) on the underlying
%              ellipsoid in [degree]
%              Southern hemisphere is signalled by negative latitude.
%
% gk2ell is meant as first step for geodetic transformations, when gk projection coordinates have to
% be transformed either to global coordinates or other projections.
%
% Note: tm2ell may also used with UTM projection parameters instead of utm2ell.
%       The difference is regarding the input data. tm2ell will work on double arrays like they are outputted
%       by ell2tm while utm2ell will only work on cell arrays like outputted by ell2utm.
%       For limitation differences regarding poles and non-standard zones see ell2tm / ell2utm.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Jan 18, 2006

%% Do some input checking

% Load input file if specified
if ischar(PRO)
    PRO=load(PRO);
end

% check input size
if ~any(ismember(size(PRO),[2 3]))
    error('Coordinate list PRO must be a nx2- or nx3-matrix!')
elseif (ismember(size(PRO,1),[2 3]))&&(~ismember(size(PRO,2),[2 3]))
    PRO=PRO';
end
n=size(PRO,1);  % Number of coordinates to transform

% Default settings
if nargin<4
    FileOut=[];
end
if nargin<3 || isempty(UND)
    UND=zeros(n,1);
elseif (numel(UND)~=n)||(~isvector(UND))
    error('Parameter ''UND'' must be a vector with the length of PRO!')
else
    UND=UND(:);
end
if nargin<2 || isempty(sys)
    sys='gk';
end

%% Load projection types
load Projections;
% Search for right projection type
if ~exist(sys,'var')
    error(['Projection type ''',sys,'''is not defined in Projections2.mat - check your definitions!'])
end
eval(['TYPE=',sys,'.type;']);
if ~strcmp(TYPE,'tm')
    error(['Projection type ''',sys,''' is not a TM projection!']);
end
eval(['m0=',sys,'.m0;']);
eval(['ellips=',sys,'.ellips;']);
eval(['rule_L0=',sys,'.rule_L0;']);
eval(['rule_easting=',sys,'.rule_easting;']);
eval(['rule_northing=',sys,'.rule_northing;']);
eval(['ID_ell=',sys,'.ID_ell;']);
eval(['ID_pro=',sys,'.ID_pro;']);
eval(['standard_L0=',sys,'.standard_L0;']);
eval(['standard_B0=',sys,'.standard_B0;']);

%% Load ellipsoids
load Ellipsoids;
if ~exist(ellips,'var')
    error(['Ellipsoid ',ellips,' is not defined in Ellipsoids.mat - check your definitions!.'])
end
eval(['ell=',ellips,';']);

%% Do the calculations
ELL=zeros(size(PRO));
eval(['PRO(:,2)=PRO(:,2)-(0',rule_northing,');']);
E=PRO(:,1);
eval(['ID=',ID_pro,';'])
eval(['B0=(',standard_B0,')/180*pi;'])

% Correction for false origin if B0 is not the equator:
if (B0 ~= 0)
    es2=(ell.a^2-ell.b^2)/ell.b^2;
    n=(ell.a-ell.b)/(ell.a+ell.b);
    V0=sqrt(1+es2*cos(B0).^2);
    eta0=sqrt(es2*cos(B0).^2);
    Lcentr = 0;
    Bf0=atan(tan(B0)./cos(V0.*Lcentr).*(1+eta0.^2/6.*(1-3*sin(B0).^2).*Lcentr.^4));
    r10=(1+n^2/4+n^4/64)*Bf0;
    r20=3/2*n*(1-n^2/8)*sin(2*Bf0);
    r30=15/16*n^2*(1-n^2/4)*sin(4*Bf0);
    r40=35/48*n^3*sin(6*Bf0);
    r50=315/512*n^4*sin(8*Bf0);
    Bkorr = ell.a/(1+n)*(r10-r20+r30-r40+r50)*m0;
    PRO(:,2) = PRO(:,2)+Bkorr;
end

n=(ell.a-ell.b)/(ell.a+ell.b);
es2=(ell.a^2-ell.b^2)/ell.b^2;

B=((1+n)/ell.a/(1+n^2/4+n^4/64)*(PRO(:,2))/m0);

b1=3/2*n*(1-9/16*n^2)*sin(2*B);
b2=n^2/16*(21-55/2*n^2)*sin(4*B);
b3=151/96*n^3*sin(6*B);
b4=1097/512*n^4*sin(8*B);

Bf=B+b1+b2+b3+b4;

% shortened Longitude:
Vf=sqrt(1+es2*cos(Bf).^2);
etaf=sqrt(es2*cos(Bf).^2);

eval(['y=PRO(:,1)-(0',rule_easting,');']);

ys=y*ell.b/m0/ell.a^2;
l=atan(Vf./cos(Bf).*sinh(ys).*(1-etaf.^4.*ys.^2/6-es2.*ys.^4/10));
eval(['ELL(:,1)=l*180/pi+',rule_L0,';'])

% Latitude:
ELL(:,2)=atan(tan(Bf).*cos(Vf.*l).*(1-etaf.^2/6.*l.^4))*180/pi;

% Height:
if (size(PRO,2)==3)
    ELL(:,3)=PRO(:,3)+UND;
end

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    if (size(ELL,2)==3)
        fprintf(fid,'%12.10f  %12.10f  %12.10f\n',ELL');
    else
        fprintf(fid,'%12.10f  %12.10f\n',ELL');
    end
    fclose(fid);
end