function [long,lat, gridlat, gridlong, aclat, aclong, header, info] = readntv2 (filename, SignConvention,SingleGridAsCell)

% READNTV2 reads a NTv2 transformation set from ASCII file
%
% [long, lat, gridlat, gridlong, aclat, aclong, header, info] 
%       = readntv2 (filename, SignConvention, SingleGridAsCell);
%
% Inputs:  filename  [string] name and path of the NTv2 file
%
%    SignConvention  0 - western longitudes and longitude shift values have positive sign 
%                    1 - western longitudes and longitude shift values have negative sign (default)
%                    By definition, NTv2 files always contain western longitudes with positive sign.
%                    Many common tools, like GeodeticToolbox, use eastern longitudes with positive
%                    sign instead. Use the SignConvention parameter to set the output style.
%
%  SingleGridAsCell  0 - return single grids as double arrays (default)
%                    1 - return single grids as cell arrays
%
% Outputs:     long  vector with interpolation positions in longitude
%               lat  vector with interpolation positions in latitude
%                       - those vectors may be used e.g. with meshgrid
%           gridlat  grid with NTv2 shift values (lat)  [unit: info.GS_TYPE]
%          gridlong  grid with NTv2 shift values (long) [unit: info.GS_TYPE]
%             aclat  grid accuracy values (lat)         [unit: info.GS_TYPE]
%            aclong  grid accuracy values (long)        [unit: info.GS_TYPE]
%                        - all grids size [lat x long]
%            header  structure containing grid header information
%              info  structure containing file information header
%
% If more than one subgrid is present, outputs except info header will be
% cell arrays.
% With only one grid, outputs will be double/structure-arrays as long as
% SingleGridAsCell is 0. When set to a value other than 0, also cells are
% outputted for easier algorithm workflow.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Jan 18, 2006

% Default settings
if (nargin<3) || isempty(SingleGridAsCell)
    SingleGridAsCell=0;
end
if (nargin<2) || isempty(SignConvention)
    SignConvention=1;
end
if (nargin<1)
    error('Input file not specified.')
end

NUM_FILE    = [];
GS_TYPE     = '';
VERSION     = '';
SYSTEM_F    = '';
SYSTEM_T    = '';
MAJOR_F     = [];
MAJOR_T     = [];
MINOR_F     = [];
MINOR_T     = [];
SUB_NAME    = '';
PARENT      = '';
CREATED     = '';
UPDATED     = '';
S_LAT       = [];
N_LAT       = [];
E_LONG      = [];
W_LONG      = [];
LAT_INC     = [];
LONG_INC    = [];
GS_COUNT    = [];
    
% Open and read NTv2 file
fid = fopen(filename);
if (fid<0)
    error('Input file couldn''t be openend.')
end

SubGridNo=0;
NewHeader=0;

while 1
    s=fgetl(fid);
    if ~ischar(s),break,end
    if strncmp(s,'NUM_FILE',8),     NUM_FILE    = str2num(s(9:end)); end;
    if strncmp(s,'GS_TYPE',7),      GS_TYPE     = strtrim(s(8:end)); end;
    if strncmp(s,'VERSION',7),      VERSION     = strtrim(s(8:end)); end;
    if strncmp(s,'SYSTEM_F',8),     SYSTEM_F    = strtrim(s(9:end)); end;
    if strncmp(s,'SYSTEM_T',8),     SYSTEM_T    = strtrim(s(9:end)); end;
    if strncmp(s,'MAJOR_F',7),      MAJOR_F     = str2num(s(8:end)); end;
    if strncmp(s,'MAJOR_T',7),      MAJOR_T     = str2num(s(8:end)); end;
    if strncmp(s,'MINOR_F',7),      MINOR_F     = str2num(s(8:end)); end;
    if strncmp(s,'MINOR_T',7),      MINOR_T     = str2num(s(8:end)); end;
    if strncmp(s,'SUB_NAME',8),     SUB_NAME    = strtrim(s(9:end));
        if (SubGridNo)
            gridlat{SubGridNo}=gridlat{SubGridNo}';
            gridlong{SubGridNo}=gridlong{SubGridNo}';
            aclat{SubGridNo}=aclat{SubGridNo}';
            aclong{SubGridNo}=aclong{SubGridNo}';
        end
        SubGridNo=SubGridNo+1;
        NewHeader=1;
    end
    if strncmp(s,'END',3)
        gridlat{SubGridNo}=gridlat{SubGridNo}';
        gridlong{SubGridNo}=gridlong{SubGridNo}';
        aclat{SubGridNo}=aclat{SubGridNo}';
        aclong{SubGridNo}=aclong{SubGridNo}';
    end
    if strncmp(s,'PARENT',6),       PARENT      = strtrim(s(7:end)); end;
    if strncmp(s,'CREATED',7),      CREATED     = strtrim(s(8:end)); end;
    if strncmp(s,'UPDATED',7),      UPDATED     = strtrim(s(8:end)); end;
    if strncmp(s,'S_LAT',5),        S_LAT       = str2num(s(6:end)); end;
    if strncmp(s,'N_LAT',5),        N_LAT       = str2num(s(6:end)); end;
    if strncmp(s,'E_LONG',6),       E_LONG      = str2num(s(7:end)); end;
    if strncmp(s,'W_LONG',6),       W_LONG      = str2num(s(7:end)); end;
    if strncmp(s,'LAT_INC',7),      LAT_INC     = str2num(s(8:end)); end;
    if strncmp(s,'LONG_INC',8),     LONG_INC    = str2num(s(9:end)); end;
    if strncmp(s,'GS_COUNT',8),     GS_COUNT    = str2num(s(9:end)); end;
    if (~isempty(str2num(s)))
        if (NewHeader)
            header{SubGridNo}=struct('SUB_NAME',SUB_NAME,'PARENT',PARENT,'CREATED',CREATED,'UPDATED',UPDATED,...
                'S_LAT',S_LAT,'N_LAT',N_LAT,'E_LONG',E_LONG,'W_LONG',W_LONG,'LAT_INC',LAT_INC,...
                'LONG_INC',LONG_INC,'GS_COUNT',GS_COUNT);
            lat{SubGridNo}=[N_LAT:-LAT_INC:S_LAT]'/3600;
            if (SignConvention==0)
                long{SubGridNo}=[W_LONG:-LONG_INC:E_LONG]'/3600;
            else
                long{SubGridNo}=[-W_LONG:LONG_INC:-E_LONG]'/3600;
                header{SubGridNo}.W_LONG=-header{SubGridNo}.W_LONG;
                header{SubGridNo}.E_LONG=-header{SubGridNo}.E_LONG;
            end
            gridlat{SubGridNo}=zeros(length(lat{SubGridNo}),length(long{SubGridNo}))';
            gridlong{SubGridNo}=zeros(length(lat{SubGridNo}),length(long{SubGridNo}))';
            aclat{SubGridNo}=zeros(length(lat{SubGridNo}),length(long{SubGridNo}))';
            aclong{SubGridNo}=zeros(length(lat{SubGridNo}),length(long{SubGridNo}))';
            NewHeader=0;
            GridValue=0;
        end
        n=str2num(s);
        gridlat{SubGridNo}(end-GridValue)=n(1);
        if (SignConvention==0)
                gridlong{SubGridNo}(end-GridValue)=n(2);
            else
                gridlong{SubGridNo}(end-GridValue)=-n(2);
        end
        aclat{SubGridNo}(end-GridValue)=n(3);
        aclong{SubGridNo}(end-GridValue)=n(4);
        GridValue=GridValue+1;
    end
end

info=struct('NUM_FILE',NUM_FILE,'GS_TYPE',GS_TYPE,'VERSION',VERSION,'SYSTEM_F',SYSTEM_F,'SYSTEM_T',SYSTEM_T,...
    'MAJOR_F',MAJOR_F,'MINOR_F',MINOR_F,'MAJOR_T',MAJOR_T,'MINOR_T',MINOR_T);

% Possibly output results as non-cells
if (SubGridNo==1 && SingleGridAsCell==0)
    lat=lat{1};
    long=long{1};
    gridlat=gridlat{1};
    gridlong=gridlong{1};
    aclat=aclat{1};
    aclong=aclong{1};
    header=header{1};
end
    