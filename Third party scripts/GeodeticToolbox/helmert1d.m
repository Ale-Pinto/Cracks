function [tp,ac,tr]=helmert1d(datum1,datum2,WithOutScale,NameToSave)

% HERLMERT1D    overdetermined cartesian 1D similarity transformation ("Helmert-Transformation")
%
% [param, accur, resid] = helmert1D(datum1,datum2,DontUseScale,SaveIt)
%
% Inputs:  datum1  n x 1 - matrix with coordinates in the origin datum (z)
%                  datum1 may also be a file name with ASCII data to be processed. No point IDs, only
%                  coordinates as if it was a matrix.
%
%          datum2  n x 1 - matrix with coordinates in the destination datum (z)
%                  datum2 may also be a file name with ASCII data to be processed. No point IDs, only
%                  coordinates as if it was a matrix.
%                  If either datum1 and/or datum2 are ASCII files, make sure that they are of same
%                  length and contain corresponding points. There is no auto-assignment of points!
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
% Outputs:  param  2 x 1 Parameter set of the 1D similarity transformation
%                      1 translations (z) in [Unit of datums]
%                      1 scale factor
%
%           accur  2 x 1 accuracy of the parameters (or scalar if scale factor is set to be 1)
%
%           resid  n x 1 - matrix with the residuals datum2 - f(datum1,param)
%
% Used to determine transformation parameters e.g. for height adjustment when at least 2 identical 
% points in both datum systems are known. Simply, this is a linear regression.
% Parameters can be used with d1trafo.m

% 09/12/11 Peter Wasmeier - Technische Universität München
% p.wasmeier@bv.tum.de

%% Argument checking and defaults

if nargin<4
    NameToSave=[];
else
    NameToSave=strtrim(NameToSave);
    if strcmp(NameToSave,'#o')
        NameToSave=[];
    end
end

if nargin<3 || isempty(WithOutScale)
    WithOutScale=0;
end

% Load input file if specified
if ischar(datum1)
    datum1=load(datum1);
end
if ischar(datum2)
    datum2=load(datum2);
end

if (size(datum1,1)==1)
    datum1=datum1'; 
end
if (size(datum2,1)==1)
    datum2=datum2'; 
end

s1=size(datum1);
s2=size(datum2);
if any(s1~=s2)
    error('The datum sets are not of equal size')
elseif any([s1(2) s2(2)]~=[1 1])
    error('At least one of the datum sets is not 1D')
elseif any([s1(1) s2(1)]<2)
    error('At least 2 points in each datum are necessary for calculating')
end

%% Approximation values and adjustment

naeh=[mean(datum2)-mean(datum1) 1];
if WithOutScale
    naeh(2)=WithOutScale;
end
WertA=[1e-8];
zaehl=0;

z0=naeh(1);
m=naeh(2);

tp=[z0 m]';

Qbb=eye(s1(1));

while 1
    A=[ones(s1(1),1),datum1];
    r=size(A,1)-size(A,2);
    w=datum2-(z0+m*datum1);
    Pbb=inv(Qbb);
    
    if WithOutScale
        A=A(:,1);
    end

     deltax=(A'*Pbb*A)\(A'*Pbb*w);
    v=A*deltax-w;
    sig0p=sqrt((v'*Pbb*v)/r);
    Qxxda=inv(A'*Pbb*A);
    Kxxda=sig0p^2*Qxxda;
    ac=sqrt(diag(Kxxda));
    zaehl=zaehl+1;
    z0=z0+deltax(1);
    if ~WithOutScale && (m+deltax(2))>1e-15     % This condition is to prevent numerical problems with m-->0
        m=m+deltax(2);
    end
    tp=[z0 m]';
    if abs(deltax(1)) < WertA(1)
        break;
    elseif zaehl>1000
        warning('Helmert1D:Too_many_iterations','Calculation not converging after 1000 iterations. I am aborting. Results may be inaccurate.')
        break;
    end
end

%% Transformation residuals
tr=datum2-(z0+m*datum1);

%% Store data

if ~isempty(NameToSave)
    load Transformations;
    if zaehl>1000
        warning('Helmert1D:Results_too_inaccurate_to_save','Results may be inaccurate and do not get stored.')
    elseif exist(NameToSave,'var') && length(NameToSave)>=2 && ~strcmp(NameToSave(end-1:end),'#o')
        warning('Helmert1D:Parameter_already_exists',['Parameter set ',NameToSave,' already exists and therefore is not stored.'])
    else
        if strcmp(NameToSave(end-1:end),'#o')
            NameToSave=NameToSave(1:end-2);
        end
        if any(regexp(NameToSave,'\W')) || any(regexp(NameToSave(1),'\d'))
            warning('Helmert1D:Parameter_name_invalid',['Name ',NameToSave,' contains invalid characters and therefore is not stored.'])
        else
            eval([NameToSave,'=tp'';']);
            save('Transformations.mat',NameToSave,'-append');
        end
    end
end
