function OUT=itrstrafo(IN,FrameIn,EpochIn,FrameOut,EpochOut,FileOut)

% ITRSTRAFO performs 3D-transformation of global point coordinate and velocity information between
%           different ITRS/ETRS - realizations (frames) using offical transformation parameter sets
%           Also works on IGS frames.
%
% OUT = itrstrafo(IN, FrameIn, EpochIn, FrameOut, EpochOut, FileOut)
%
% Also necessary:   Trafo_ITRF.mat   (see beneath)
%
% Inputs:   IN  nx3 - matrix with coordinates or nx6 - matrix with coordinates and velocities to be 
%               transformed. 3|6 x n-matrices are allowed, be careful with 3|6 x 3|6-matrices!
%               For nx3 matrices the velocity component is set to all zeros per default
%               Dimension: [m] and [m/year]
%               IN may also be a file name with ASCII data to be processed. No point IDs, only
%               coordinates as if it was a matrix.
%
%      FrameIn  The frame of the input coordinates as string
%               This frame must be defined in Trafo_ITRF.mat and look like one of the following:
%               For all frames prior to 2000 it is 'ITRFxx' or 'ETRFxx' with xx being the shortened
%               year number, e.g. 'ITRF96'.
%               Far all frames in 2000 or later, it is 'ITRFxxxx' or 'ETRFxxxx' with complete year
%               number, e.g. 'ETRF2000'.
%               IGS frames are to be used as 'IGSxxxx' with complete year number also.
%               IGS2008 to NAD83 is possible, while NAD83 frames are to be called
%               'NAD83(2011)', 'NAD83(MA11)' or 'NAD83(PA11)'.
%               Default if omitted or set to [] is 'ITRF2000'
%
%      EpochIn  The epoch the input coordinates are defined in. Input is a decimal year number.
%               If left out or empty, the reference epoch of the input FrameIn will be used for ITRS
%               and the corresponding year number epoch for ETRS frames.
%
%     FrameOut  The frame of the output coordinates as string
%               See FrameIn. Default if omitted or set to [] is 'ITRF2008'.
%
%     EpochOut  The epoch the output coordinates are defined in. Input is a decimal year number.
%               If left out or empty, the reference epoch of the input FrameOut will be used for ITRS
%               and the corresponding year number epoch for ETRS frames.
%
%      FileOut  File to write the output to. If omitted, no output file is generated.
%
% Output:  OUT  nx6-matrix with the transformed coordinates and velocities.
%
% If not only a reference frame change, but also a epoch change is to be computed, velocity information
% needs to be considered. If no velocity information is given, only the movement rates as given in
% the predefined transformation parameter sets will be used, so even for this points the output will
% contain velocity information.
% The base of the transformation is a 3D 14-parameter-transformation with small rotations.
% Parameters in Trafo_ITRF.mat are offical parameters from IGN homepage, http://itrf.ensg.ign.fr/
% There is also an online transformation tool with the same functionality like this function file:
% http://www.epncb.oma.be/_productsservices/coord_trans/
%
% Comment on IGS frames:
% It is possible to transform IGS2005 to IGS2008 and back using this function. The parameters can be
% found here: http://igscb.jpl.nasa.gov/pipermail/igsmail/2011/006346.html
% There is no transformation between ITRF2008 and IGS2008 as it is assumed to be Zero.
%
% Comment on NAD frames:
% It is only possible to transform IGS2008 to NAD83 and back using this function. The parameters can be
% found here: https://www.ngs.noaa.gov/CORS/coords.shtml
% The results will differ from official NGS trnsformation tool "HTDP software" as this tool also
% obeys deformation models which the pure transformation can't.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Jan 18, 2006

%% Argument checking and defaults
load Trafo_ITRF Trafo_ITRF;

if nargin<6
    FileOut=[];
end
if nargin<5
    EpochOut=[];
end
if nargin<4 || isempty(FrameOut)
    FrameOut='ITRF2008';
elseif ~any(any(strcmp(FrameOut,Trafo_ITRF(:,[1 2 5]))))
    error('Parameter ''FrameOut'' must be defined as ITRF frame in Trafo_ITRF.mat!')
end
if nargin<3
    EpochIn=[]; 
end
if nargin<2 || isempty(FrameIn)
    FrameIn='ITRF2000';
elseif ~any(any(strcmp(FrameIn,Trafo_ITRF(:,[1 2 5]))))
    error('Parameter ''FrameIn'' must be defined as ITRF frame in Trafo_ITRF.mat!')
end

% Load input file if specified
if ischar(IN)
    IN=load(IN);
end

if isempty(EpochIn) && any(strcmp(FrameIn,Trafo_ITRF(:,1)))
    EpochIn=Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,1)),4};
elseif isempty(EpochIn) && any(strcmp(FrameIn,Trafo_ITRF(:,5)))
    EpochIn=Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,5)),7};
end
if isempty(EpochOut) && any(strcmp(FrameOut,Trafo_ITRF(:,1)))
    EpochOut=Trafo_ITRF{strcmp(FrameOut,Trafo_ITRF(:,1)),4};
elseif isempty(EpochOut) && any(strcmp(FrameOut,Trafo_ITRF(:,5)))
    EpochOut=Trafo_ITRF{strcmp(FrameOut,Trafo_ITRF(:,5)),7};
end
if ~any(ismember(size(IN),[3 6]))
    error('Coordinate list IN must be a nx3- or nx6-matrix!')
elseif (ismember(size(IN,1),[3 6]))&&(~ismember(size(IN,2),[3 6]))
    IN=IN';
end
if size(IN,2)==3
   IN=[IN zeros(size(IN))];     % add zero velocities 
end

%% Find the right calculation path

while 1
    if strcmp(FrameIn,FrameOut)
        OUT=epoch_change(IN,EpochIn,EpochOut);
        break;
    elseif any(strcmp(FrameIn,Trafo_ITRF(:,5)))
        % Start frame is not ITRS
        if (strncmp(FrameIn,'ETRF',4))  % StartFrame is in ETRS - first transform to ETRS in 1989
            IN=epoch_change(IN,EpochIn,1989);
            EpochIn=1989;
            % transform it to ITRS
            IN=trafo_etrs(IN,-Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,5)),6});
        elseif (strncmp(FrameIn,'NAD',3))  % StartFrame is in NAD
            % transform it to ITRS at input epoch
            IN=trafoNAD(IN,-Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,5)),6},EpochIn,1997);
        else
            error('Transformation between these frames is not specified by parameters.')
        end
        FrameIn=Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,5)),1};
        % then go on with the next "elseif" statements
    elseif (any(strcmp(FrameIn,Trafo_ITRF(:,1))))&&(any(strcmp(FrameOut,Trafo_ITRF(:,5))))
        % Start frame is in ITRS and target frame is not - first transform to target frame in ITRS
        IN=itrstrafo(IN,FrameIn,EpochIn,Trafo_ITRF{strcmp(FrameOut,Trafo_ITRF(:,5)),1},EpochIn);
        if (strncmp(FrameOut,'ETRF',4))  % TargetFrame is in ETRS
            % change it to epoch 1989.0 in ITRS
            IN=epoch_change(IN,EpochIn,1989);
            EpochIn=1989;
            % then transform to ETRS
            IN=trafo_etrs(IN,Trafo_ITRF{strcmp(FrameOut,Trafo_ITRF(:,5)),6});
        elseif (strncmp(FrameOut,'NAD',3))  % Target Frame is in NAD
            % change it to target epoch in ITRS
            IN=epoch_change(IN,EpochIn,EpochOut);
            EpochIn=EpochOut;
            % then transform to NAD
            IN=trafoNAD(IN,Trafo_ITRF{strcmp(FrameOut,Trafo_ITRF(:,5)),6},EpochIn,1997);
        else
            error('Transformation between these frames is not specified by parameters.')
        end
        FrameIn=FrameOut;
    elseif Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,1)),4}<2000
        % Start frame before 2000 -> transform to 2000 in any case
        IN=trafo(IN,Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,1)),3},EpochIn,Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,1)),4});
        FrameIn='ITRF2000';
    elseif (strcmp(FrameIn,'ITRF2000'))&&(Trafo_ITRF{strcmp(FrameOut,Trafo_ITRF(:,1)),4}<2000)
        % Start frame is 2000 and destination frame is before 2000 -> transform with inverted parameters
        IN=trafo(IN,-Trafo_ITRF{strcmp(FrameOut,Trafo_ITRF(:,1)),3},EpochIn,Trafo_ITRF{strcmp(FrameOut,Trafo_ITRF(:,1)),4});
        FrameIn=FrameOut;
    elseif (str2double(FrameIn(end-3:end))>=2000)&&...
           (str2double(FrameOut(end-3:end))>str2double(FrameIn(end-3:end)))
        % Start frame is 2000 or later and destination frame is after start frame -> transform 1 step towards
        % destination frame
        % This also works for IGS frames.
        IN=trafo(IN,Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,1)),3},EpochIn,Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,1)),4});
        FrameIn=Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,1)),2};
    elseif (str2double(FrameIn(end-3:end))>2000)&&...
           (str2double(FrameOut(end-3:end))<str2double(FrameIn(end-3:end)))
        % Start frame is 2005 or later and destination frame is before start frame -> transform 1 step towards
        % destination frame
        % This also works for IGS frames.
        IN=trafo(IN,-Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,2)),3},EpochIn,Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,2)),4});
        FrameIn=Trafo_ITRF{strcmp(FrameIn,Trafo_ITRF(:,2)),1};
    else
        error('Transformation between these ITRF/EUREF or IGS frames is not specified by parameters.')
    end
end

%% Write output to file if specified

if ~isempty(FileOut)
    fid=fopen(FileOut,'w+');
    fprintf(fid,'%12.6f  %12.6f  %12.6f  %12.6f  %12.6f  %12.6f\n',OUT');
    fclose(fid);
end

%% Helper functions

function C=epoch_change(C,EpochIn,EpochOut)
% Apply only epoch change - just use velocities:
C(:,1:3)=C(:,1:3)+C(:,4:6).*(EpochOut-EpochIn);

function C=trafo(C,p,EpochIn,RefEpoch)
% Apply the transformation in the epoch of the input frame
p(1:3)=p(1:3)*1e-3;
p(4:6)=p(4:6)/1e3/180/3600*pi;
p(7)=p(7)*1e-9;
p(8:10)=p(8:10)*1e-3;
p(11:13)=p(4:6)/1e3/180/3600*pi;
p(14)=p(14)*1e-9;
p(1:7)=p(1:7)+p(8:14)*(EpochIn-RefEpoch);
for i=1:size(C,1)
    C(i,1:3)=[C(i,1:3)'+p(1:3)'+[p(7) -p(6) p(5);p(6) p(7) -p(4);-p(5) p(4) p(7)]*C(i,1:3)']';
    C(i,4:6)=[C(i,4:6)'+p(8:10)'+p(14)*C(i,1:3)'+[0 -p(13) p(12);p(13) 0 -p(11);-p(12) p(11) 0]*C(i,1:3)']';
end

function C=trafoNAD(C,p,EpochIn,RefEpoch)
% Apply the transformation in the epoch of the input frame
p(1:3)=p(1:3)*1e-3;
p(4:6)=p(4:6)/1e3/180/3600*pi;
p(7)=1+p(7)*1e-9;
p(8:10)=p(8:10)*1e-3;
p(11:13)=p(4:6)/1e3/180/3600*pi;
p(14)=p(14)*1e-9;
p(1:7)=p(1:7)+p(8:14)*(EpochIn-RefEpoch);
for i=1:size(C,1)
    C(i,1:3)=[p(1:3)'+[p(7) p(6) -p(5);-p(6) p(7) p(4);p(5) -p(4) p(7)]*C(i,1:3)']';
    C(i,4:6)=[p(8:10)'+p(14)*C(i,1:3)'+[0 p(13) -p(12);-p(13) 0 p(11);p(12) -p(11) 0]*C(i,1:3)']';
end

function C=trafo_etrs(C,p)
p(1:3)=p(1:3)*1e-3;
p(4:6)=p(4:6)/1e3/180/3600*pi;
for i=1:size(C,1)
    C(i,4:6)=[C(i,4:6)'+[0 -p(6) p(5);p(6) 0 -p(4);-p(5) p(4) 0]*C(i,1:3)']';
    C(i,1:3)=C(i,1:3)+p(1:3);
end

