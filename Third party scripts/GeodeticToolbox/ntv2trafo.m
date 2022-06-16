function [OUT,UsedGridName]=ntv2trafo(IN,gridlat,gridlong,header,direction,FileOut)

% NTV2TRAFO performs a NTV2 grid transformation
%
% [OUT, UsedGridName] = ntv2trafo(IN, gridlat, gridlong, header, FileOut)
%
% Inputs:        IN  nx2 - matrix of points to be transformed [Long Lat] [°]
%                    2xn-matrices are allowed, but be careful with 2x2-matrices.
%                    IN may also be a file name with ASCII data to be processed. No point IDs, only
%                    coordinates as if it was a matrix.
%
%           gridlat  cell array of grids with NTv2 shift values (lat)  ["]
%          gridlong  cell array of grid with NTv2 shift values (long) ["]
%            header  cell array of structures containing grid header information
%                    NTV2TRAFO works on inverted longitude sign convention as produced by READNTV2
%                    i.e. western longitudes have negative sign (standardized NTv2 transformation
%                    grids show positive sign for western longitudes).
%                    Please use READNTV2 to get the proper inputs for NTV2TRAFO.
%                    With only one grid entered, the input variables may also be double/struct
%                    matrices instead of cell arrays.
%
%         direction  0 if transformation is in standard direction. (Default if omitted or set to [])
%                    1 if reverse transformation is to be performed.
%                    Per definition, shift values are defined in the source system. So they are meant
%                    to transform Sys1 -> Sys2, but not in reverse direction. If reverse direction is
%                    required, this will be done by this function using an iterative algorithm. See
%                    comments below.
%                   
%           FileOut  File to write the output OUT to. If omitted, no output file is generated.
%
% Outputs:      OUT  nx2 - matrix with the transformed output variables [Long Lat] [°]
%
%      UsedGridName  nx1 - cell array with the name of the sub_grid which was used for each
%                    transformed point. The names are taken from SUB_NAME field in the header.
%                    If the point is out of specified grids, '* NOT SPECIFIED *' is returned.
%
% For each point it is individually searched for the proper grid. If more than one grid is convenient,
% the algorithm takes the one with the lowest size of mesh (mesh diagonal).
% If a point is outside of the specified grids, a warning is thrown and the point is returned
% untransformed. Use 'warning off NTv2:NoGridAvailable' to suppress the warnings.
%
% Note: NTv2 is an gridwise affine transformation with linear interpolation between knot points.
%       This inherently means that there will be interpolation unsteadiness (a kink) at the knot
%       positions. As a result, different coordinates very close to a knot point, which lie in
%       different meshes may be mapped to identical target coordinates (depending on the mesh width
%       of the used subgrid).
%       As converse argument, when performing a backward transformation ([Sys1a->]Sys2->Sys1b), it may 
%       occur that other coordinates are found for Sys1b than the starting ones of Sys1a in such cases.
%       The error may be up to 0.1" which would lead to about 3 meters.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Jan 18, 2006

%% Do input checking, set defaults and adjust vectors

if nargin<6
    FileOut=[];
end
if nargin<5 || isempty(direction)
    direction = 0;
end
if nargin<4
    error('Not all parameters have been specified.');
end
if iscell(gridlat)
    if  any(diff([length(gridlat) length(gridlong) length(header)]))
        error('Input cell arrays must be of equal length.')
    end
    for i=1:length(gridlat)
        if any(size(gridlat{i})~=size(gridlong{i}))
            error('GridLat and GridLong dimensions do not match.');
        end
    end
else
    if any(size(gridlat)~=size(gridlong))
        error('GridLat and GridLong dimensions do not match.');
    end
    gridlat={gridlat};gridlong={gridlong};header={header};
end

% Load input file if specified
if ischar(IN) % Load file
    IN=load(IN);
end

if     (size(IN,1)~=2)&&(size(IN,2)~=2)
    error('Coordinate list IN must be a nx2-matrix!')
elseif (size(IN,1)==2)&&(size(IN,2)~=2)
    IN=IN';
end

IN=IN*3600;
OUT=zeros(size(IN));
UsedGridName=cell(size(IN,1),1);

if (direction)
    IN0=IN;
end
dIN=1;

%% Calculate the transformation
ssh=[];
while (any(abs(dIN)>1e-10))
    % Find the correct grid to work in and do the calculations
    for k=1:size(IN,1);
        g_inc=inf;
        UsedGrid=0;
        for i=1:length(gridlat)
            if (header{i}.W_LONG <= IN(k,1))&&(IN(k,1) <= header{i}.E_LONG) &&...
                    (header{i}.S_LAT <= IN(k,2))&&(IN(k,2) <= header{i}.N_LAT)
                width=header{i}.LAT_INC^2+header{i}.LONG_INC^2;
                if (width)<g_inc
                    g_inc=width;
                    UsedGrid=i;
                end
            end
        end
        if UsedGrid==0
            warning('NTv2:NoGridAvailable',['Transformation point ',num2str(k),' is out of the specified grids.']);
            OUT(k,:)=IN(k,:);
            UsedGridName{k}='* NOT SPECIFIED *';
            continue;
        end
        UsedGridName{k}=header{UsedGrid}.SUB_NAME;
        fcol=(IN(k,1)-header{UsedGrid}.W_LONG)/header{UsedGrid}.LONG_INC;
        frow=(IN(k,2)-header{UsedGrid}.S_LAT)/header{UsedGrid}.LAT_INC;
        col=floor(fcol);
        row=floor(frow);
        if col==size(gridlat{UsedGrid},2)-1
            col=col-1;
        end
        if row==size(gridlat{UsedGrid},1)-1
            row=row-1;
        end
        dx=fcol-col;
        dy=frow-row;
        svlat=(1-dx)*(1-dy)*gridlat{UsedGrid}(size(gridlat{UsedGrid},1)-row,col+1)+...
            dx *(1-dy)*gridlat{UsedGrid}(size(gridlat{UsedGrid},1)-row,col+2)+...
            dx *   dy *gridlat{UsedGrid}(size(gridlat{UsedGrid},1)-row-1,col+2)+...
            (1-dx)*   dy *gridlat{UsedGrid}(size(gridlat{UsedGrid},1)-row-1,col+1);
        svlong=(1-dx)*(1-dy)*gridlong{UsedGrid}(size(gridlat{UsedGrid},1)-row,col+1)+...
            dx *(1-dy)*gridlong{UsedGrid}(size(gridlat{UsedGrid},1)-row,col+2)+...
            dx *   dy *gridlong{UsedGrid}(size(gridlat{UsedGrid},1)-row-1,col+2)+...
            (1-dx)*   dy *gridlong{UsedGrid}(size(gridlat{UsedGrid},1)-row-1,col+1);
        ssh(k,:)=[svlong svlat];
        OUT(k,:)=(IN(k,:)+[svlong svlat]);
    end
    if (~direction)
        break
    end
    dIN=OUT-IN0;
    IN=IN-dIN;
end
if (direction)
    OUT=IN;
end
OUT=OUT/3600;
ssh
%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%12.10f  %12.10f\n',OUT');
    fclose(fid);
end