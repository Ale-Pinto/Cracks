function PRO=ell2utm(ELL,ell,format,fixedZone,UND,FileOut)

% ELL2UTM performs transformation from ellipsoidal coordinates to a utm mapping projection
%
% PRO=ell2utm(ELL,ell,format,fixedZone,UND,FileOut)
%
% Also necessary:   Ellipsoids.mat   (see beneath)
%
% Inputs:  ELL  coordinates on ellipsoid as nx2-matrix (longitude, latitude) [degree]
%               2xn-matrices are allowed. Be careful with 2x2-matrices!
%               If for (some) points ellipsoidic height is available, ELL may be a nx3-matrix also.
%               Southern hemisphere is signalled by negative latitude.
%               ELL may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%          ell  the underlying ellipsoid as string in lower case letters
%               Default if omitted or set to [] is 'grs80'.
%               See Ellipsoids.m for details.
%
%       format  the desired UTM coordinate output format
%               Default if omitted or set to [] is 'grid3'
%               Possible formats:
%                   gridX    32U460000.123 5498765.123           X digits  (string)
%                   mgrsX    32UMV6000098765                    no digits  (string)
%                mgrsXold    32UMF6000098765                    no digits  (string)
%                            X is the number of grid digits or MGRS integer places (0-5)                          
%
%    fixedZone  If points of neighboured zones shall be calculated all with respect to the same central
%               meridian, one can force to use a single zone by giving the zone number.
%               Default if omitted or set to [] is 0 which means calculate individual zone per point.
%
%          UND  undulation values from geoid model to calculate projected height from ellipsoidal height.
%               If omitted or set to [], no correction is done on the height in PRO.
%               Ellipsoidal height = Projected height + Undulation value
%
%      FileOut  File to write the output to. If omitted, no output file is generated.
%
% Outputs: PRO  nx1-array of strings. In this case, height is outputted as double 
%               in second cell column.
%
% ell2utm is meant as last step for geodetic transformations, when global coordinates or other
% projections have to be transformed to utm projection coordinates.
%
% Note: Naturally, in most cases ell2utm() and ell2tm('utm') give identical results when the same 
%       ellipsoids are used. However, ell2utm() uses standard output formats (which contain at least one 
%       letter showing the vertical band) and can handle polar regions with UPS mapping and special zone
%       width variations which ell2tm can't.
%       ell2tm instead is just doing a tm mapping with utm parameters and ignoring false northing on
%       southern hemisphere for unambiguousness while having no vertical zone identifier.
%       While for a point on southern hemisphere ell2utm shows e.g. 33H459741.966 6151265.479,
%       ell2tm will produce [33459741.9661022 -3848734.5208106]. This might be favourable for further 
%       calculation steps (with the mentioned limitations).

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Aug 25, 2011

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
nct=size(ELL,1);  % Number of coordinates to transform
if nargin<6
    FileOut=[];
end
if      nargin<5 || isempty(UND)
    UND=zeros(nct,1);
elseif (numel(UND)~=nct)||(~isvector(UND))
    error('Parameter ''UND'' must be a vector with the length of ELL!')
else
    UND=UND(:)';
end
if nargin<4 || isempty(fixedZone) 
    fixedZone=0;
elseif (~isscalar(fixedZone))||(fixedZone<0)||(fixedZone>60)
    error ('Parameter ''fixedZone'' must be a scalar in [0 60]')
end
fixedZone=round(fixedZone);
if nargin<3 || isempty(format) 
    format='grid3';
elseif ~(any(strcmp(format,{'grid0','grid1','grid2','grid3','grid4','grid5','mgrs0','mgrs1','mgrs2','mgrs3','mgrs4',...
        'mgrs5','mgrs0old','mgrs1old','mgrs2old','mgrs3old','mgrs4old','mgrs5old'})))
    error('Parameter ''format'' must be any of ''gridX'', ''mgrsX'' or ''mgrsXold'' (X in [0 5])''!')
end
if nargin<2 || isempty(ell) 
    ell='grs80';
end

%% Load ellipsoids
load Ellipsoids;
if ~exist(ell,'var')
    error(['Ellipsoid ',ell,' is not defined in Ellipsoids.mat - check your definitions!.'])
end
eval(['ell=',ell,';']);

%% Do calculations

ID=zeros(nct,2);
L0=zeros(nct,1);

% Find the correct zone.
for i=1:nct
    if ELL(i,2)>=84  % north pole
        if (ELL(i,1)<0)&&(ELL(i,2)<90)
            ID(i,1:2)=[0 25];  
            L0(i)=0;
        else
            ID(i,1:2)=[0 26];
            L0(i)=0;
        end
    elseif ELL(i,2)<-80  % south pole
        if (ELL(i,1)<0)&&(ELL(i,2)>-90)
            ID(i,1:2)=[0 1];
            L0(i)=0;
        else
            ID(i,1:2)=[0 2];
            L0(i)=0;
        end
    elseif ELL(i,2)>=56 && ELL(i,2)<=64 && ELL(i,1)>=0 && ELL(i,1)<3   % V31
        ID(i,1:2)=[31 22];
        L0(i)=3;
    elseif ELL(i,2)>=56 && ELL(i,2)<=64 && ELL(i,1)>=3 && ELL(i,1)<12   % V32
        ID(i,1:2)=[32 22];
        L0(i)=9;
    elseif ELL(i,2)>=72 && ELL(i,2)<=84 && ELL(i,1)>=0 && ELL(i,1)<9   % X31
        ID(i,1:2)=[31 24];
        L0(i)=3;
    elseif ELL(i,2)>=72 && ELL(i,2)<=84 && ELL(i,1)>=9 && ELL(i,1)<21   % X33
        ID(i,1:2)=[33 24];
        L0(i)=15;
    elseif ELL(i,2)>=72 && ELL(i,2)<=84 && ELL(i,1)>=21 && ELL(i,1)<33   % X35
        ID(i,1:2)=[35 24];
        L0(i)=27;
    elseif ELL(i,2)>=72 && ELL(i,2)<=84 && ELL(i,1)>=33 && ELL(i,1)<42   % X37
        ID(i,1:2)=[37 24];
        L0(i)=39;
    else  % all others
        ID(i,1)=floor(ELL(i,1)/6)+31;
        if ELL(:,2)>=72
            ID(i,2)=24;
        else
           ID(i,2)=floor(ELL(i,2)/8)+13;
           if ID(i,2)>=9
               ID(i,2)=ID(i,2)+1;
           end
           if ID(i,2)>=15
               ID(i,2)=ID(i,2)+1;
           end
        end
          L0(i)=ID(i,1)*6-183;
    end
end

if (fixedZone)
   ID(:,1)=fixedZone; 
   L0(:)=ID(:,1)*6-183;
end

PROs=zeros(size(ELL));
rho=180/pi;

%% handle all points except the pole regions

% 1. eccentricity
e2=(ell.a^2-ell.b^2)/ell.a^2;
% 2. eccentricity
es2=(ell.a^2-ell.b^2)/ell.b^2;

if any(ID(:,1)>0)
    B=ELL(ID(:,1)>0,2)/rho;
    L=(ELL(ID(:,1)>0,1)-L0(ID(:,1)>0))/rho;
    m0=0.9996;
    
    V=sqrt(1+es2*cos(B).^2);
    eta=sqrt(es2*cos(B).^2);

    Bf=atan(tan(B)./cos(V.*L).*(1+eta.^2/6.*(1-3*sin(B).^2).*L.^4));
    Vf=sqrt(1+es2*cos(Bf).^2);
    etaf=sqrt(es2*cos(Bf).^2);
    n=(ell.a-ell.b)/(ell.a+ell.b);

    % numerical series for ordinate:
    r1=(1+n^2/4+n^4/64)*Bf;
    r2=3/2*n*(1-n^2/8)*sin(2*Bf);
    r3=15/16*n^2*(1-n^2/4)*sin(4*Bf);
    r4=35/48*n^3*sin(6*Bf);
    r5=315/512*n^4*sin(8*Bf);

    PROs(ID(:,1)>0,2)=ell.a/(1+n)*(r1-r2+r3-r4+r5)*m0;
    % false northing on southern hemisphere
    PROs(B<0,2)=PROs(B<0,2)+10e6;

    % abscissa :
    ys=asinh(tan(L).*cos(Bf)./Vf.*(1+etaf.^2.*L.^2.*cos(Bf).^2.*(etaf.^2/6+L.^2/10)));
    y=m0*ell.a^2/ell.b*ys;
    % false easting
    PROs(ID(:,1)>0,1)=y+5e5;
end
%% deal the poles - ups mapping
if any(ID(:,1)==0)
    m0=0.994;
    woN=and(ID(:,1)==0,ID(:,2)>24);
    woS=and(ID(:,1)==0,ID(:,2)<3);

    B=ELL(woN,2)/rho;
    L=ELL(woN,1)/rho;
    C0=2*ell.a/sqrt(1-e2)*((1-sqrt(e2))/(1+sqrt(e2)))^(sqrt(e2)/2);
    tanz2=((1+sqrt(e2)*sin(B))./(1-sqrt(e2)*sin(B))).^(sqrt(e2)/2).*tan(pi/4-B/2);
    R=m0*C0*tanz2;
    PROs(woN,1)=2e6+R.*sin(L); % north pole
    PROs(woN,2)=2e6-R.*cos(L); % north pole

    B=-ELL(woS,2)/rho;
    L=ELL(woS,1)/rho;
    C0=2*ell.a/sqrt(1-e2)*((1-sqrt(e2))/(1+sqrt(e2)))^(sqrt(e2)/2);
    tanz2=((1+sqrt(e2)*sin(B))./(1-sqrt(e2)*sin(B))).^(sqrt(e2)/2).*tan(pi/4-B/2);
    R=m0*C0*tanz2;
    PROs(woS,1)=2e6+R.*sin(L); % south pole
    PROs(woS,2)=2e6+R.*cos(L); % south pole
end
%% Finish

% Height:
if (size(ELL,2)==3),
    PROs(:,3)=ELL(:,3)-UND;
end

% Now define output format
places=str2double(format(5));
switch format(1:4)
    case 'grid'
        for i=1:nct
            if ID(i,1)==0
                PRO{i,1}=sprintf(['%c %0.',num2str(places),'f %0.',num2str(places),'f'],char(ID(i,2)+64),PROs(i,1:2));
            else
                PRO{i,1}=sprintf(['%d%c %0.',num2str(places),'f %0.',num2str(places),'f'],ID(i,1),char(ID(i,2)+64),PROs(i,1:2));
            end
        end
        if (size(ELL,2)==3),
            for i=1:nct
                PRO{i,2}=PROs(i,3);
            end
        end
    case 'mgrs'
        mgrs_east_table=[19:26; 1:8; 10:14 16:18]+64;
        if length(format)>5 % old notation
            mgrs_north_table=[18:22 1:8 10:14 16:17;
                12:14 16:22 1:8 10:11]+64;
        else
            mgrs_north_table=[6:8 10:14 16:22 1:5;
                1:8 10:14 16:22]+64;
        end
        mgrs_pole_east_table=[1:3 6:8 10:12 16:18;
            26:-1:24 21:-1:16 12:-1:10]+64;
        mgrs_northpole_north_table=[8 10:14 16; 7:-1:1]+64;
        mgrs_southpole_north_table=[14 16:26; 13:-1:10 8:-1:1]+64;

        for i=1:nct
            if ID(i,1)==0
                Z=floor((PROs(i,1)-2e6)/1e5);
                if (Z>=0) 
                    EC=mgrs_pole_east_table(1,Z+1);
                else
                    EC=mgrs_pole_east_table(2,abs(Z));
                end
                Z=floor((PROs(i,2)-2e6)/1e5);
                if ID(i,1)==0 && ID(i,2)<3  % south pole
                    if (Z>=0) 
                        NC=mgrs_southpole_north_table(1,Z+1);
                    else
                        NC=mgrs_southpole_north_table(2,abs(Z));
                    end
                elseif ID(i,1)==0 && ID(i,2)>24  % north pole
                    if (Z>=0) 
                        NC=mgrs_northpole_north_table(1,Z+1);
                    else
                        NC=mgrs_northpole_north_table(2,abs(Z));
                    end
                end
            else
                EC=mgrs_east_table(mod(ID(i,1),3)+1,floor(PROs(i,1)/1e5));  % east
                NC=mgrs_north_table(mod(ID(i,1),2)+1,mod(floor(PROs(i,2)/1e5),20)+1);   % north
            end
            CO=floor(mod(PROs(i,1:2),1e5)/(10^(5-places)));
            if ID(i,1)==0
                PRO{i,1}=sprintf(['%c%c%c %0',num2str(places),'d%0',num2str(places),'d'],char(ID(i,2)+64),char(EC),char(NC),CO);
            else
                PRO{i,1}=sprintf(['%d%c%c%c %0',num2str(places),'d%0',num2str(places),'d'],ID(i,1),char(ID(i,2)+64),char(EC),char(NC),CO);
            end
        end
        if (size(ELL,2)==3),
            for i=1:nct
                PRO{i,2}=PROs(i,3);
            end
        end
end

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    if size(PRO,2)==1
        fprintf(fid,'%s\n',PRO{:,1});
    else
        for i=1:size(PRO,1)
            fprintf(fid,'%s  %8.4f\n',PRO{i,1},PRO{i,2});
        end
    end
    fclose(fid);
end