function [tp,ac,tr]=helmertprojective2d(datum1,datum2,NameToSave)

% HERLMERTPROJECTIVE2D    overdetermined cartesian 2D projective transformation
%
% [param, accur, resid] = helmertprojective2D(datum1,datum2,SaveIt)
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
%          SaveIt  string with the name to save the resulting parameters in Transformations.mat.
%                  Make sure only to use characters which are allowed in Matlab variable names
%                  (e.g. no spaces, umlaute, leading numbers etc.)
%                  If left out or empty, no storage is done.
%                  If the iteration is not converging, no storage is done and a warning is thrown.
%                  If the name to store already is existing, it is not overwritten automatically.
%                  To overwrite an existing name, add '#o' to the name, e.g. 'wgs84_to_local#o'.
%
% Outputs:  param  8 x 1 Parameter set of the 2D projective transformation
%
%           accur  8 x 1 accuracy of the parameters
%
%           resid  n x 2 - matrix with the residuals datum2 - f(datum1,param)
%
% Used to calculate projective transformation parameters when at least 4 identical points in both systems
% are known. An projective transformation is mainly used for image rectification.
% Parameters can be used with d2projectivetrafo.m

% 11/12/21 Peter Wasmeier - Technische Universität München
% p.wasmeier@bv.tum.de
 
%% Argument checking and defaults

if nargin<3
    NameToSave=[];
else
    NameToSave=strtrim(NameToSave);
    if strcmp(NameToSave,'#o')
        NameToSave=[];
    end
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
elseif any([s1(1) s2(1)]<4)
    error('At least 4 points in each datum are necessary for calculating')
end

%% Projective transformation:

% Make nearly singular matrix warning an error
s = warning('error','MATLAB:nearlySingularMatrix');

G=zeros(2*s1(1),8);
t=zeros(2*s2(1),1);
for i=1:s1(1)
   G(2*i-1:2*i,:)=[datum1(i,:) 1 0 0 0 -datum1(i,:).*datum2(i,1);
                   0 0 0 datum1(i,:) 1 -datum1(i,:).*datum2(i,2)];
   t(2*i-1:2*i)=datum2(i,:)';
end
tp=(G'*G)\(G'*t);
if (size(G,1)>8)
    v=G*tp-t;
    sig0p=sqrt((v'*v)/(size(G,1)-8));
    try
        ac=sqrt(diag(sig0p^2*inv(G'*G)));
    catch
        ac=sqrt(diag(sig0p^2*pinv(G'*G)));
    end
else
    ac=zeros(8,1);
end

% Restore warning
warning(s);

%% Transformation residuals
idz=zeros(s1);
for i=1:s1(1)
    idz(i,:)=[tp(1)*datum1(i,1)+tp(2)*datum1(i,2)+tp(3) tp(4)*datum1(i,1)+tp(5)*datum1(i,2)+tp(6)]/(tp(7)*datum1(i,1)+tp(8)*datum1(i,2)+1);
end
tr=datum2-idz;

%% Store data

if ~isempty(NameToSave)
    load Transformations;
    if exist(NameToSave,'var') && length(NameToSave)>=2 && ~strcmp(NameToSave(end-1:end),'#o')
        warning('Helmert2D:Parameter_already_exists',['Parameter set ',NameToSave,' already exists and therefore is not stored.'])
    else
        if strcmp(NameToSave(end-1:end),'#o')
            NameToSave=NameToSave(1:end-2);
        end
        if any(regexp(NameToSave,'\W')) || any(regexp(NameToSave(1),'\d'))
            warning('Helmert2D:Parameter_name_invalid',['Name ',NameToSave,' contains invalid characters and therefore is not stored.'])
        else
            eval([NameToSave,'=[',num2str(tp'),'];']);
            save('Transformations.mat',NameToSave,'-append');
        end
    end
end

