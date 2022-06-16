function [tp,rc,ac,tr]=helmert2d(datum1,datum2,Type,WithOutScale,NameToSave)

% HERLMERT2D    overdetermined cartesian 2D similarity transformation ("Helmert-Transformation")
%
% [param, rotcent, accur, resid] = helmert2D(datum1,datum2,Type,DontUseScale,SaveIt)
%
% Inputs:  datum1  n x 2 - matrix with coordinates in the origin datum (x y)
%                  datum1 may also be a file name with ASCII data to be processed. No point IDs, only
%                  coordinates as if it was a matrix.
%
%          datum2  n x 2 - matrix with coordinates in the destination datum (x y)
%                  datum2 may also be a file name with ASCII data to be processed. No point IDs, only
%                  coordinates as if it was a matrix.
%                  If either datum1 and/or datum2 are ASCII files, make sure that they are of same
%                  length and contain corresponding points. There is no auto-assignment of points!
%
%            Type  is either '4p' for 4-parameter-transformation (standard case with origin in [0 0])
%                  or '6p' for 6-parameter-transformation with rotation center at centroid of
%                  datum1.
%                  Default is '4p' which is also choosen if Type is not a string (e.g. []).
%
%    DontUseScale  if this is not 0, do not calculate scale factor but set it to the inputted value
%                  Default: 0 (Use scale)
%
%          SaveIt  string with the name to save the resulting parameters in Transformations.mat.
%                  Make sure only to use characters which are allowed in Matlab variable names
%                  (e.g. no spaces, umlaute, leading numbers etc.)
%                  If left out or empty, no storage is done.
%                  If the iteration is not converging, no storage is done and a warning is thrown.
%                  If the name to store already is existing, it is not overwritten automatically.
%                  To overwrite an existing name, add '#o' to the name, e.g. 'wgs84_to_local#o'.
%
% Outputs:  param  4 x 1 Parameter set of the 2D similarity transformation
%                      2 translations (x y) in [Unit of datums]
%                      1 rotation (ez) in [rad]
%                      1 scale factor
%
%         rotcent  2 x 1 vector of rotation center in datum1 [x y]
%
%           accur  4 x 1 accuracy of the parameters (or 3 x 1 if scale factor is set to be 1)
%
%           resid  n x 2 - matrix with the residuals datum2 - f(datum1,param)
%
% Used to determine transformation parameters e.g. for cadastral purposes (transforming local system
% to 2D mapping system) when at least 2 identical points in both datum systems are known.
% Parameters can be used with d2trafo.m

% 09/12/11 Peter Wasmeier - Technische Universität München
% p.wasmeier@bv.tum.de
% 04/17/15 Andrea Pinna - University degli Studi di Cagliari
% andrea.pinna@unica.it

%% Argument checking and defaults

if nargin<5
    NameToSave=[];
else
    NameToSave=strtrim(NameToSave);
    if strcmp(NameToSave,'#o')
        NameToSave=[];
    end
end

if nargin<4 || isempty(WithOutScale)
    WithOutScale=0;
end

if nargin<3 || ~ischar(Type) || isempty(Type)
    Type='4p'; 
end

% Load input file if specified
if ischar(datum1)
    datum1=load(datum1);
end
if ischar(datum2)
    datum2=load(datum2);
end

if (size(datum1,1)==2)&&(size(datum1,2)~=2)
    datum1=datum1'; 
end
if (size(datum2,1)==2)&&(size(datum2,2)~=2)
    datum2=datum2'; 
end

s1=size(datum1);
s2=size(datum2);
if any(s1~=s2)
    error('The datum sets are not of equal size')
elseif any([s1(2) s2(2)]~=[2 2])
    error('At least one of the datum sets is not 2D')
elseif any([s1(1) s2(1)]<2)
    error('At least 2 points in each datum are necessary for calculating')
end

switch Type
    case '4p'
        rc=[0 0];
    case '6p'
        rc=mean(datum1);
    otherwise
        error ('Transformation type needs to be ''4p'' or ''6p''.')
end

%% Approcimation values and adjustment

naeh=[0 0 0 1];
if WithOutScale
    naeh(4)=WithOutScale;
end
WertA=[1e-8 1e-8];
zaehl=0;

x0=naeh(1);
y0=naeh(2);
ez=naeh(3);
m=naeh(4);

tp=[x0 y0 ez m];

% This will only be used, if point accuracies were given - not used yet
% Qbb=eye(2*s1(1));

while(1)
    A=zeros(2*s1(1),4);
    w=zeros(2*s1(1),1);
    
    % Compute sine and cosine of angle - speeds up computation
	cosez = cos(ez);
	sinez = sin(ez); 
    
    A(1:2:end,1)=1;
    A(2:2:end,2)=1;
    
    A(1:2:end,3)=m*(-sinez*(datum1(:,1)-rc(1))+cosez*(datum1(:,2)-rc(2)));
    A(1:2:end,4)=cosez*(datum1(:,1)-rc(1))+sinez*(datum1(:,2)-rc(2));
    A(2:2:end,3)=m*(-cosez*(datum1(:,1)-rc(1))-sinez*(datum1(:,2)-rc(2)));
    A(2:2:end,4)=-sinez*(datum1(:,1)-rc(1))+cosez*(datum1(:,2)-rc(2));
    
    w(1:2:end,1)=-rc(1)+datum2(:,1)-x0-m*(cosez*(datum1(:,1)-rc(1))+sinez*(datum1(:,2)-rc(2)));
    w(2:2:end,1)=-rc(2)+datum2(:,2)-y0-m*(-sinez*(datum1(:,1)-rc(1))+cosez*(datum1(:,2)-rc(2)));

    if WithOutScale
        A=A(:,1:3);
    end
    
    warning off;
    r=size(A,1)-size(A,2);
    Pbb = speye(2*s1(1));   % Accuracy of points is not used yet
    APbbA = A'*Pbb*A;
    deltax=APbbA\(A'*Pbb*w);
    v=A*deltax-w;
    sig0p=sqrt((v'*Pbb*v)/r);
    Kxxda=sig0p^2*inv(APbbA);
    ac=sqrt(diag(Kxxda));
    warning on;

    testv=sqrt((deltax(1)^2+deltax(2)^2)/2);
    testd=deltax(3);
    zaehl=zaehl+1;
    x0=x0+deltax(1);
    y0=y0+deltax(2);
    ez=ez+deltax(3);
    if ~WithOutScale && (m+deltax(4))>1e-15     % This condition is to prevent numerical problems with m-->0
        m=m+deltax(4);
    end
    tp=[x0 y0 ez m]';
    if abs(testv) < WertA(1) && abs(testd) < WertA(2)
        break;
    elseif zaehl>1000
        warning('Helmert2D:Too_many_iterations','Calculation not converging after 1000 iterations. I am aborting. Results may be inaccurate.')
        break;
    end
end

%% Transformation residuals
idz=zeros(s1);
idz(:,2)=rc(2)+tp(2)+tp(4)*(-sin(tp(3))*(datum1(:,1)-rc(1))+cos(tp(3))*(datum1(:,2)-rc(2)));
idz(:,1)=rc(1)+tp(1)+tp(4)*(cos(tp(3))*(datum1(:,1)-rc(1))+sin(tp(3))*(datum1(:,2)-rc(2)));

tr=datum2-idz;

%% Store data

if ~isempty(NameToSave)
    load Transformations;
    if zaehl>1000
        warning('Helmert2D:Results_too_inaccurate_to_save','Results may be inaccurate and do not get stored.')
    elseif exist(NameToSave,'var') && length(NameToSave)>=2 && ~strcmp(NameToSave(end-1:end),'#o')
        warning('Helmert2D:Parameter_already_exists',['Parameter set ',NameToSave,' already exists and therefore is not stored.'])
    else
        if strcmp(NameToSave(end-1:end),'#o')
            NameToSave=NameToSave(1:end-2);
        end
        if any(regexp(NameToSave,'\W')) || any(regexp(NameToSave(1),'\d'))
            warning('Helmert2D:Parameter_name_invalid',['Name ',NameToSave,' contains invalid characters and therefore is not stored.'])
        else
            eval([NameToSave,'=num2str(tp'');']);
            save('Transformations.mat',NameToSave,'-append');
        end
    end
end
