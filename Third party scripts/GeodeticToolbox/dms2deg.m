function deg=dms2deg(d,m,s,OnlyDnegative,FileOut)

% DMS2DEG convert degree, minute, second values to double degree value
%
% deg = dms2deg(d,m,s,FileOut)
%
% Inputs:         d  integer (array) of degrees
%                 m  integer (array) of minutes
%                 s  double (array) of seconds
%                    d may also be a file name with ASCII data to be processed. No point IDs, only
%                    [degree - minute - second] values as if it was a matrix. m and s are ignored
%                    in this case.
%     OnlyDnegative  0 = negative inputs show negative sign at d, m and s (default if omitted or set
%                        to [])
%                    1 = if only the degree value d is negative, treat also m and s as negative
%                        inputs regardless of their sign.
%                        If d=0, the sign of m is obeyed; if also m=0, the sign of s is taken.
%           FileOut  File to write the output to. If omitted, no output file is generated.
%
% Outputs:      deg  double (array) of degree values
%
% Works on degrees with 360° for full circle.
% See also dms2degrees from the Mathworks mapping toolbox.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Jan 18, 2006

%%
% Defaults
if nargin<5
    FileOut=[];
end
if nargin<4 || isempty(OnlyDnegative)
    OnlyDnegative=0;
end
if nargin<3 || isempty(s)
    s=zeros(size(d));
end
if nargin<2 || isempty(m)
    m=zeros(size(d));
end

% Load input file if specified
if ischar(d)
    d=load(d);
    s=d(:,3);
    m=d(:,2);
    d=d(:,1);
end

if OnlyDnegative
   m(d~=0)=abs(m(d~=0));
   s((d~=0)&(m~=0))=abs(s((d~=0)&(m~=0)));
   m(d<0)=-m(d<0);
   s((m<0)|(d<0))=-s(m<0);
end

%% Calculation

deg=d+m/60+s/3600;

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%14.10f\n',deg);
    fclose(fid);
end