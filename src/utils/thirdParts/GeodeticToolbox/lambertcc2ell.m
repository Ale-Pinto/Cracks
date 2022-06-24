function ELL=lambertcc2ell(PRO,sys,UND,FileOut)

% LAMBERTCC2ELL performs transformation from a conic Lambert mapping projection to ellipsoidal coordinates
%               using two standard parallels
%
% ELL=lambertcc2ell(PRO,sys,UND,FileOut)
%
% Also necessary:   Projections.mat   Ellipsoids.mat   (see beneath)
%
% Inputs:  PRO  Coordinates in projection system as nx2-matrix (abscissa and ordinate vaules).
%               2xn-matrices are allowed. Be careful with 2x2-matrices!
%               If for (some) points orthometric height is available, PRO may be a nx3-matrix also.
%               PRO may also be a file name with ASCII data to be processed. No point IDs, only
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
% Outputs: ELL  nx2- resp. nx3- matrix with longitude, latitude (and height) on the underlying
%               ellipsoid in [degree]
%               Southern hemisphere is signalled by negative latitude.
%
% lambertcc2ell is meant as first step for geodetic transformations, when projection coordinates have to
% be transformed either to global coordinates or other projections.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Aug 26, 2011

%% Do some input checking

% Load input file if specified
if ischar(PRO)
    PRO=load(PRO);
end

% Input size and defaults
if ~any(ismember(size(PRO),[2 3]))
    error('Coordinate list PRO must be a nx2- or nx3-matrix!')
elseif (ismember(size(PRO,1),[2 3]))&&(~ismember(size(PRO,2),[2 3]))
    PRO=PRO';
end
n=size(PRO,1);  % Number of coordinates to transform
if nargin<4
    FileOut=[];
end
if nargin<3 || isempty(UND)
    UND=zeros(n,1);
elseif (numel(UND)~=n)||(~isvector(UND))
    error('Parameter ''UND'' must be a vector with the size length of PRO!')
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
    error(['Projection type ''',sys,''' is not a Lambert CC2 projection!']);
end
eval(['LAT=',sys,'.lat;']);
eval(['ellips=',sys,'.ellips;']);
eval(['ORell=',sys,'.ORell;']);
eval(['ORproj=',sys,'.ORproj;']);
rho=180/pi;
LAT=LAT/rho;
ORell=ORell/rho;

%% Load ellipsoids
load Ellipsoids;
if ~exist(ellips,'var')
    error(['Ellipsoid ',ellips,' is not defined in Ellipsoids.mat - check your definitions!.'])
end
eval(['ell=',ellips,';']);

%% Do the calculations
ELL=zeros(size(PRO));

Ns=PRO(:,2)-ORproj(2);
Es=PRO(:,1)-ORproj(1);

e=sqrt((ell.a^2-ell.b^2)/ell.a^2);
t1=tan(pi/4-LAT(1)/2)/((1-e*sin(LAT(1)))/(1+e*sin(LAT(1))))^(e/2);
t2=tan(pi/4-LAT(2)/2)/((1-e*sin(LAT(2)))/(1+e*sin(LAT(2))))^(e/2);
t0=tan(pi/4-ORell(2)/2)/((1-e*sin(ORell(2)))/(1+e*sin(ORell(2))))^(e/2);
m1=cos(LAT(1))/sqrt(1-e^2*sin(LAT(1))^2);
m2=cos(LAT(2))/sqrt(1-e^2*sin(LAT(2))^2);
n=(log(m1)-log(m2))/(log(t1)-log(t2));
F=m1/(n*t1^n);
r0=ell.a*F*t0^n;

rs=sign(n)*sqrt(Es.^2+(r0-Ns).^2);
ts=(rs./ell.a./F).^(1/n);
gs=atan2(Es,r0-Ns);
ELL(:,1)=gs./n+ORell(1);

phi=pi/2-2*atan(ts);
diff=1e6;
while any(diff>1e-15)
    phi_neu=pi/2-2*atan(ts.*((1-e*sin(phi))./(1+e*sin(phi))).^(e/2));
    diff=abs(phi_neu-phi);
    phi=phi_neu;
end

ELL(:,2)=phi;
ELL=ELL*rho;

% Height:
if (size(PRO,2)==3),
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