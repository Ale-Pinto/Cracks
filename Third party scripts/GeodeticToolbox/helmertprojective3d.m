function [tp,ac,tr]=helmertprojective3d(datum1,datum2,NameToSave)

% HERLMERTPROJECTIVE3D    overdetermined cartesian 3D projective transformation to 2D
%
% [param, accur, resid] = helmertprojective3D(datum1,datum2,SaveIt)
%
% Inputs:  datum1  n x 3 - matrix with coordinates in the 3D origin datum (x y z)
%                  datum1 may also be a file name with ASCII data to be processed. No point IDs, only
%                  coordinates as if it was a matrix.
%
%          datum2  n x 2 - matrix with coordinates in the 2D destination datum (x y)
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
% Outputs:  param  11 x 1 Parameter set of the 3D projective transformation
%
%           accur  11 x 1 accuracy of the parameters
%
%           resid  n x 2 - matrix with the residuals datum2 - f(datum1,param)
%
% Used to calculate projective transformation parameters when at least 6 identical points in both systems
% are known. An 3D projective transformation is mainly used for image mapping.
% Parameters can be used with d3projectivetrafo.m

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

if (size(datum1,1)==3)&&(size(datum1,2)~=3)
    datum1=datum1'; 
end
if (size(datum2,1)==2)&&(size(datum2,2)~=2)
    datum2=datum2'; 
end

s1=size(datum1);
s2=size(datum2);
if s1(1) ~= s2(1)
    error('The datum sets are not of equal length')
elseif s1(2)~=3
    error('Datum1 needs to be 3D.')
elseif s2(2)~=2
    error('Datum2 needs to be 2D.')
elseif any([s1(1) s2(1)]<6)
    error('At least 6 points in each datum are necessary for calculating')
end

%% Projective transformation:

% Make nearly singular matrix warning an error
s = warning('error','MATLAB:nearlySingularMatrix');

G=zeros(2*s1(1),11);
t=zeros(2*s2(1),1);
for i=1:s1(1)
   G(2*i-1:2*i,:)=[datum1(i,:) 1 0 0 0 0 -datum1(i,:).*datum2(i,1);
                   0 0 0 0 datum1(i,:) 1 -datum1(i,:).*datum2(i,2)];
   t(2*i-1:2*i)=datum2(i,:)';
end
tp=(G'*G)\(G'*t);
if (size(G,1)>11)
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
idz=zeros(s2);
for i=1:s1(1)
    idz(i,:)=[tp(1)*datum1(i,1)+tp(2)*datum1(i,2)+tp(3)*datum1(i,3)+tp(4) ...
        tp(5)*datum1(i,1)+tp(6)*datum1(i,2)+tp(7)*datum1(i,3)+tp(8)]...
        /(tp(9)*datum1(i,1)+tp(10)*datum1(i,2)+tp(11)*datum1(i,3)+1);
end
tr=datum2-idz;

%% Store data

if ~isempty(NameToSave)
    load Transformations;
    if exist(NameToSave,'var') && length(NameToSave)>=2 && ~strcmp(NameToSave(end-1:end),'#o')
        warning('Helmert3D:Parameter_already_exists',['Parameter set ',NameToSave,' already exists and therefore is not stored.'])
    else
        if strcmp(NameToSave(end-1:end),'#o')
            NameToSave=NameToSave(1:end-2);
        end
        if any(regexp(NameToSave,'\W')) || any(regexp(NameToSave(1),'\d'))
            warning('Helmert3D:Parameter_name_invalid',['Name ',NameToSave,' contains invalid characters and therefore is not stored.'])
        else
            eval([NameToSave,'=[',num2str(tp'),'];']);
            save('Transformations.mat',NameToSave,'-append');
        end
    end
end
