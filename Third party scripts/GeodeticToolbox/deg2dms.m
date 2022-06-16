function [g,m,s]=deg2dms(d, FileOut)

% DEG2DMS convert double degree value to degree, minute, second
%
% [d,m,s] = deg2dms(deg, FileOut)
%
% Inputs:       deg  double (array) of degree values
%                    deg may also be a file name with ASCII data to be processed. No point IDs, only
%                    degree values as if it was a vector.
%           FileOut  File to write the output to. If omitted, no output file is generated.
%
% Outputs:        d  integer (array) of degrees
%                 m  integer (array) of minutes
%                 s  double (array) of seconds
%
% Works on degrees with 360° for full circle.
% Negative inputs will produce all negative output values.
% See also degrees2dms from the Mathworks mapping toolbox.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Jan 18, 2006

%%
% Load input file if specified
if ischar(d)
    d=load(d);
end

if nargin<2
    FileOut=[];
end

% Calculations
g=fix(d);
m=fix((d-g)*60);
s=((d-g)*60-m)*60;

% Necessary corrections due to working precision
s(abs((round(s)-s)/3600)<eps)=round(s(abs((round(s)-s)/3600)<eps));
m(s==-60)=m(s==-60)-1;
m(s==60)=m(s==60)+1;
s(s==-60)=0;
s(s==60)=0;

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%3d  %2df  %12.10f\n',[g;m;s]);
    fclose(fid);
end