function ELL=utm2ell(PRO,ell,UND,UseMGRSold,FileOut)

% UTM2ELL performs transformation from a utm mapping projection to ellipsoidal coordinates
%
% ELL=utm2ell(PRO,ell,UND,UseMGRSold,FileOut)
%
% Also necessary:   Projections2.mat   Ellipsoids.mat   (see beneath)
%
% Inputs:  PRO  Cell array nx1 with one string per line containing coordinates
%               If for (some) points orthometric height is available, PRO may be a nx2-cell array
%               with doubles as height information
%               PRO may also be a file name with ASCII data to be processed. No point IDs, only UTM
%               coordinates with possible height separated by blanks.
%
%               Possible formats are:  
%                    grid    32U460000.123 5498765.123   arbitrary digits  (string) or
%                            32U 460000.123 5498765.123  arbitrary digits  (string)
%                   mgrsX    32UMV6000098765                    no digits  (string)
%                mgrsXold    32UMF6000098765                    no digits  (string)
%                            X is the number of MGRS integer places (0-5) 
%
%                The appropriate format is detected automatically, except mgrsXold.
%                This will only be used when UseMGRSold is set to a value other than 0.
%                The format in the input PRO must not vary!
%
%          ell  the underlying ellipsoid as string in lower case letters
%               Standard if omitted or set to [] is 'grs80'.
%               See Ellipsoids.m for details.                 
%
%          UND  undulation values from geoid model to calculate projected height from ellipsoidal height.
%               If omitted or set to [], no correction is done on the height in ELL.
%               Ellipsoidal height = Projected height + Undulation value
%
%   UseMGRSold  When MGRS is used in old format, this input parameter has te be set to a value
%               other than 0. Default if omitted or set to [] is 0.
%
%      FileOut  File to write the output to. If omitted, no output file is generated.
%
% Outputs: ELL  nx2- resp. nx3- matrix with longitude, latitude (and height) on the underlying
%               ellipsoid in [degree]
%               Southern hemisphere is signalled by negative latitude.
%
% utm2ell is meant as first step for geodetic transformations, when utm projection coordinates have to
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
    fid=fopen(PRO,'r');
    i=1;
    while 1 % Pre-allocation counter
        line=fgetl(fid);
        if ~ischar(line)
            break
        end
        i=i+1;
    end
    Coo=cell(i,1);
    PRO=cell(i,1);
    i=1;
    while 1
        line=fgetl(fid);
        if ~ischar(line)
            break
        end
        Coo{i,1}=strtrim(line);
        i=i+1;
    end
    fclose(fid);
    for i=1:length(Coo)
        letters=find(and(double(Coo{i})>=65,double(Coo{i})<=91));
        if length(letters)==1
            tpos=2;
        elseif length(letters)==3
            tpos=1;
        else
            error('Format description of input file is not valid (1 or 3 letters needed)!')
        end
        temp=strtrim(Coo{i}(letters(end)+1:end));
        spac=findstr(temp,' ');
        if length(spac)>=tpos,
            PRO{i,1}=[Coo{i}(1:letters(end)) strtrim(temp(1:spac(tpos)))];
            PRO{i,2}=str2num(temp(spac(tpos)+1:end));
        else
            PRO{i,1}=Coo{i};
        end
    end
end

% Check input size
if ~any(ismember(size(PRO),[1 2]))
    error('Coordinate list PRO must be nx1 or nx2 when being a cell array!')
elseif (ismember(size(PRO,1),[1 2]))&&(~ismember(size(PRO,2),[1 2]))
    PRO=PRO';
end

% Check input format and search for letters in UTM strings
letters=and(double(PRO{1,1})>=65,double(PRO{1,1})<=91);
woletters=find(letters);
if sum(letters)==1
    format='grid';
elseif sum(letters)==3
    format='mgrs';
    places=(length(PRO{1,1})-woletters(end))/2;
    format=[format,num2str(places)];
else
    error('Format description of cell array is not valid (1 or 3 letters needed)!')
end

% Allocate variables
nct=size(PRO,1);  % Number of coordinates to transform
if size(PRO,2)==1
    P=zeros(size(PRO,1),2);
else
    P=zeros(size(PRO,1),3);
end
ID=zeros(size(PRO,1),2);
L0=zeros(size(PRO,1),1);

% Default settings
if nargin<5
    FileOut=[];
end
if nargin<4 || isempty(UseMGRSold)
    UseMGRSold=0;
end
if nargin<3 || isempty(UND)
    UND=zeros(nct,1);
elseif (numel(UND)~=nct)||(~isvector(UND))
    error('Parameter ''UND'' must be a vector with the length of PRO!')
else
    UND=UND(:)';
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

%% Undo the formatting
switch format(1:4)
    case 'grid'
        for i=1:nct
            letters=find(and(double(PRO{i,1})>=65,double(PRO{i,1})<=91));
            if ~any(ismember(letters,[1 2 3]))
                error(['Input value #',num2str(i),'is not in grid format.']);
            end
            ThisID=str2num(PRO{i,1}(1:letters(1)-1));
            if (isempty(ThisID))||(ThisID==0)
                ID(i,1)=0;    % Polar region
                L0=0;
            else
                ID(i,1)=ThisID;
                L0(i)=ID(i,1)*6-183;
            end
            ID(i,2)=double(PRO{i,1}(letters(1)));
            P(i,1:2)=str2num(PRO{i,1}(letters(1)+1:end));
            if (ID(i,1)~=0) && (ID(i,2)<78) % southern hemisphere
                P(i,2)=P(i,2)-1e7;
            end
            if size(PRO,2)==2
                P(i,3)=PRO{i,2};
            end
        end
    case 'mgrs'
        easting_table=[65:72; 74:78 80:82; 83:90; -4e5:1e5:3e5];
        pole_easting_table=[65:67 70:72 74:76 80:85 88:90; 0:1e5:17e5];
        northing_zone=[67:72 74:78 80:88];
        pole_northing_table=[65:72 74:78 80:90;-7e5:1e5:16e5;-12e5:1e5:11e5];
        if (UseMGRSold==0)
            northing_table=[65:72 74:78 80:86];
        else
            northing_table=[76:78 80:86 65:72 74:75];
        end
        for i=1:nct
            letters=find(and(double(PRO{i,1})>=65,double(PRO{i,1})<=90));
            if ~any(ismember(letters(1),[1 2 3]))||length(letters)~=3
                error(['Input value #',num2str(i),'is not in mgrs format.']);
            end
            ThisID=str2num(PRO{i,1}(1:letters(1)-1));
            if (isempty(ThisID))||(ThisID==0)
                ID(i,1)=0;    % Polar region
                L0=0;
                ID(i,2)=double(PRO{i,1}(letters(1)));
                Pshort=[str2num(PRO{i,1}(letters(3)+1:letters(3)+places)) str2num(PRO{i,1}(letters(3)+1+places:end))];
                [woi,woj]=find(pole_easting_table==double(PRO{i,1}(letters(2))));
                if any(double(PRO{i,1}(letters(1)))==[65 89])  % western
                    P(i,1)=2e6+Pshort(1)*10^(5-places)+pole_easting_table(2,woj)-18e5;
                else
                    P(i,1)=2e6+Pshort(1)*10^(5-places)+pole_easting_table(2,woj);
                end
                [woi,woj]=find(pole_northing_table==double(PRO{i,1}(letters(3))));
                if any(double(PRO{i,1}(letters(1)))==[65 66])  % south pole
                    P(i,2)=2e6+Pshort(2)*10^(5-places)+pole_northing_table(3,woj);
                else
                    P(i,2)=2e6+Pshort(2)*10^(5-places)+pole_northing_table(2,woj);
                end
            else
                ID(i,1)=ThisID;
                L0(i)=ID(i,1)*6-183;
                ID(i,2)=double(PRO{i,1}(letters(1)));
                Pshort=[str2num(PRO{i,1}(letters(3)+1:letters(3)+places)) str2num(PRO{i,1}(letters(3)+1+places:end))];
                [woi,woj]=find(easting_table==double(PRO{i,1}(letters(2))));
                P(i,1)=5e5+easting_table(4,woj)+Pshort(1)*10^(5-places);
                approx_north=885000*(find(northing_zone==ID(i,2))-10.5);
                shortened_north=(find(northing_table==double(PRO{i,1}(letters(3))))-1)*1e5;
                if mod(ID(i,1),2)==0    % even zone - starting with 'F'
                    shortened_north=shortened_north-5e5;
                end
                diffn=approx_north-shortened_north;
                multn=round(diffn/2e6)*2e6;
                P(i,2)=multn+shortened_north+Pshort(2)*10^(5-places);
            end
            if size(PRO,2)==2
                P(i,3)=PRO{i,2};
            end
        end
end

%% Do the calculations

ELL=zeros(size(P));

% 1. eccentricity
e2=(ell.a^2-ell.b^2)/ell.a^2;
% 2. eccentricity
es2=(ell.a^2-ell.b^2)/ell.b^2;

% Transverse mercator - all except poles
if any(ID(:,1)>0)
    y=P(ID(:,1)>0,1)-5e5;
    x=P(ID(:,1)>0,2);
    m0=0.9996;

    n=(ell.a-ell.b)/(ell.a+ell.b);
    es2=(ell.a^2-ell.b^2)/ell.b^2;

    B=(1+n)/ell.a/(1+n^2/4+n^4/64)*x/m0;

    b1=3/2*n*(1-9/16*n^2)*sin(2*B);
    b2=n^2/16*(21-55/2*n^2)*sin(4*B);
    b3=151/96*n^3*sin(6*B);
    b4=1097/512*n^4*sin(8*B);

    Bf=B+b1+b2+b3+b4;

    % shortened Longitude:
    Vf=sqrt(1+es2*cos(Bf).^2);
    etaf=sqrt(es2*cos(Bf).^2);

    ys=y*ell.b/m0/ell.a^2;
    l=atan(Vf./cos(Bf).*sinh(ys).*(1-etaf.^4.*ys.^2/6-es2.*ys.^4/10));
    ELL(ID(:,1)>0,1)=l*180/pi-183+ID(ID(:,1)>0,1)*6;

    % Latitude:
    ELL(ID(:,1)>0,2)=atan(tan(Bf).*cos(Vf.*l).*(1-etaf.^2/6.*l.^4))*180/pi;
end

if any(ID(:,1)==0)    % now handle the poles - ups
    m0=0.994;
    C0=2*ell.a/sqrt(1-e2)*((1-sqrt(e2))/(1+sqrt(e2)))^(sqrt(e2)/2);
    As=e2/2+5*e2^2/24+e2^3/12+13*e2^4/360;
    Bs=7*e2^2/48+29*e2^3/240+811*e2^4/11520;
    Cs=7*e2^3/120+81*e2^4/1120;
    Ds=4279*e2^4/161280;
    
    PolePoints=find(ID(:,1)==0);
    for i=1:length(PolePoints)
        y=P(PolePoints(i),1)-2e6;
        x=P(PolePoints(i),2)-2e6;
        if (ID(PolePoints(i),2)>88) % north pole
            L=atan2(y,-x);
        else
            L=atan2(y,x);
        end
        if (y==0)&&(x==0)       % exactly the pole
            B=pi/2;
        else
            if (y==0)
                R=abs(x);
            elseif (x==0)
                R=abs(y);
            else
                R=abs(y/sin(L));
            end
            xi=pi/2-2*atan(R/m0/C0);
            B=xi+As*sin(2*xi)+Bs*sin(4*xi)+Cs*sin(6*xi)+Ds*sin(8*xi);
        end
        if (ID(PolePoints(i),2)<88)
            B=-B;
        end
        ELL(PolePoints(i),1:2)=[L B]*180/pi;
    end
end

% Height:
if (size(P,2)==3)
    ELL(:,3)=P(:,3)+UND;
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