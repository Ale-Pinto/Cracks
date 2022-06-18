% Trafo_ITRF.MAT
%
% Mat-File containing parameters and definitions for ITRF/ETRF or IGS transformations
%
% Parameters in Trafo_ITRF.mat are offical parameters from IGN homepage, http://itrf.ensg.ign.fr/
% and http://etrs89.ensg.ign.fr/, IGS parameters can be found at
% http://igscb.jpl.nasa.gov/pipermail/igsmail/2011/006346.html 
%
% This mat-File contains one cell arrays named Trafo_ITRF of n x 7 - size
% There is one line per transformation parameter set from one epoch to the following. Before 2000,
% there are seperate parameter sets to transform to 2000 in each frame; after 2000, the parameters
% are consecutive. The last entry always is the newest frame; of course here no parameters for the
% transformation to a future frame can be given.
% The columns are as follows:
% Column 1  -   Source frame in ITRF as string
% Column 2  -   Destination frame in ITRF as string
% Column 3  -   1x14 double transformation parameters from source to destination frame in
%               [dx dy dz rx ry rz scale ddx ddy ddz dry dry drz dscale]
%               where the latter seven are the changing rates.
%               Dimensions: [mm],[mas = .001"],[ppb = 10^-9] and [mm/y], [mas/y], [ppb/y]
% Column 4  -   Reference epoch of the transformation parameter set as double
% Column 5  -   corresponding ETRF frame as string (if existing)
% Column 6  -   1 x 6 double transformation parameters from source frame to corresponding ETRS frame
%               as defined in the ETRS transformation Memo V.8, Tables 3 and 4 and the EUREF
%               technical note 1.
%               [dx dy dz drx dry drz]. Dimensions [mm], [mas/y]
%               - OR -
%               1x14 double transformation parameters from source frame to corresponding NAD frame
%               as given in https://www.ngs.noaa.gov/CORS/coords.shtml
%               [dx dy dz rx ry rz scale ddx ddy ddz dry dry drz dscale]
%               where the latter seven are the changing rates.
%               Dimensions: [mm],[mas = .001"],[ppb = 10^-9] and [mm/y], [mas/y], [ppb/y]
% Column 7  -   Standard epoch of the ETRF frame as double
%
% The parameters are up to date until Nov-2018.
% More recent parameter sets may be simply added as a new line.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Jan 18, 2006