function u=rescorr(XYZ,P,tr,mode,varargin)

% RESCORR calculates residual corrections of a set of transformed points
%
% u=rescorr(XYZ,P,tr,mode,Param1,Param2,...,FileOut)
%
% Inputs:  XYZ  n x {1,2,3} -matrix of transformed points to correct. {1,2,3} x n-matrices are 
%               allowed, be careful with matrices of all sizes < 4!
%               XYZ may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%            P  m x {1,2,3} -matrix of the supporting points with same dimension like XYZ
%               P may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%           tr  residuals. This must be the size of P (m x 1,2,3)
%
%         mode  Algorithm (string) to correct residuals. Following modes are specified:
%               'dist'   arithmetic mean weighted by eucledian distance by 1/(s^k+C)[default]
%                        Param1 = k (weighting factor)
%                        Param2 = C (smoothing factor)
%                        e.g.  k = 1 - linear distance weighting [default]
%                              k = 2 - inverse squared weighting
%                              C = 0 - no smoothing [default]*
%                              C > 0 - smoothing summand (reduces relative differences)
%                                  
%               'mq2'    multiquadratic interpolation of 2nd order planes (Hardy 1971)
%                        Param1 = G (smoothing factor)
%                              G = 0 no smoothing [default]
%                              G > 0 smoothing summand (reduces relative differences)
%                   
%      FileOut  File to write the output OUT to. Input as Param2 or Param3 in varargin list.
%               If omitted, no output file is generated.
%
% Output:  u  n x {1,2,3}-matrix with the interpolated residuals.
%             The residuals are NOT added automatically.
%
% Note: You can use the residuals given by helmert3d or helmert 2d to correct points transformed  
%       with d3trafo or d2trafo in an adjacency preserving way.
%       While you can use transformation parameters param (Sys1 -> Sys2) from helmertXd also to perform 
%       the backwards transformation (Sys2 -> Sys1), you may use the residuals resid only to correct
%       points in Sys2. To obtain residuals also in Sys2, you need to execute another helmertXd
%       (Sys2 -> Sys1) first.
%    *  Even if C is entered to be 0 in 'dist' mode, it internally is altered to be "eps". This is 
%       done to prevent the correction values of the supporting points to become NaN due to the 
%       inverse distance of "0".

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Nov 18, 2011

%% Do input checking, set defaults and adjust vectors

FileOut=[];

if nargin<4 || isempty(mode)
    mode='dist';
elseif ~ischar(mode)
    error('Parameter ''mode'' must be a char expression.')
end
if  nargin<3
    error('Not all necessary input parameters are specified.')
end

% Load input file if specified
if ischar(XYZ)
    XYZ=load(XYZ);
end
if ischar(P)
    P=load(P);
end

if ~any(ismember(size(XYZ),[1 2 3]))
    error('Coordinate list ELL must be a nx{1,2,3}-matrix!')
elseif (ismember(size(XYZ,1),[1 2 3]))&&(~ismember(size(XYZ,2),[1 2 3]))
    XYZ=XYZ';
end
if size(P)~=size(tr)
    error('The sizes if supporting points and residuals must match! ');
end
if size(XYZ,2)~=size(P,2)
    error('Dimension of residuals ind points to interpolate must match!');
end
if ~(any(strcmp(mode,{'dist','mq2'})))
    error('Parameter ''mode'' must be any of ''dist'' or ''mq2''!')
end
% number of coordinate triplets to transform
n=size(XYZ,1);
m=size(P,1);

%% Calculate
switch mode
    case 'dist'
        if length(varargin)>=3 && ~isempty(varargin{3})
            FileOut=varargin{3};
        end
        if nargin<6 || varargin{2}==0 || isempty(varargin{2})
            C=eps;
        else
            C=varargin{2};
        end
        if nargin<5 || isempty(varargin{1})
            k=1;
        else
            k=varargin{1};
        end
        for i=1:n
            p=1./(sqrt(sum((P-repmat(XYZ(i,:),size(P,1),1)).^2,2)).^(k)+C);
            u(i,:)=(p'*tr)./sum(p);
        end
    case 'mq2'
         if ~isempty(varargin{2})
             FileOut=varargin{2};
         end
        if nargin<5 || isempty(varargin{1})
            G=0;
        else
            G=varargin{1};
        end
        S=zeros(size(P,1));
        for j=1:m-1
            for k=j+1:m
                S(j,k)=sqrt(sum((P(j,:)-P(k,:)).^2)+G);
                S(k,j)=S(j,k);
            end
        end
        invS=inv(S)*tr;
        for i=1:n
            s=sqrt(sum((P-repmat(XYZ(i,:),size(P,1),1)).^2,2)+G);
            u(i,:)=s'*invS;
        end
end

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    if size(P,2)==1
        fprintf(fid,'%12.10f\n',u);
    elseif size(P,2)==2
        fprintf(fid,'%12.10f  %12.10f\n',u')
    elseif size(P,2)==2
        fprintf(fid,'%12.10f  %12.10f   %12.10f\n',u')
    end
    fclose(fid);
end