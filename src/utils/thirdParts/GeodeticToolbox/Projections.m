% PROJECTIONS.MAT
%
% Mat-File containing projection definitions.
% This mat-File contains cell arrays named by the projection in lower case letters.
%
% Each cell-array has a 'type'-field which sets the type of mapping projection:
%
% 1. 'tm' - Transverse Mercator projection
%
% Usually, different projection zones are used with individual central meridians to keep deformations
% small at the edges. Therefore, the zone to be used is signalled by a zone identifier (ID) which
% is part of the easting coordinate to keep easting values unambiguous.
%
% The projection cell array fields are all strings, except m0:
%
%                   m0  The scaling factor of the central meridian (scalar)
%               ellips  The underlying ellipsoid as string in lower case letters
%                       The ellipsoids are stored in the mat-File "Ellipsoids.mat" also as
%                       cell-arrays named by the ellipsoid, e.g. 'bessel1841' or 'wgs84'.
%                       See "help Ellipsoids" for further details.
%              rule_L0  The rule to translate the zone identifier ID for the projection zone to
%                       the central meridian longitude when transforming projected coordinates to
%                       ellipsoidal coordinates (tm2ell).
%                       Allows following variables to be used: ID
%                       The zone identifier is taken from the abscissa coordinate of the projection
%                       and is transformed into a addition on the calculated longitude-per-zone.
%                       For example, for GK-projection the rule is "ID*3" due to 3°-zone width 
%                       starting at Greenwich.
%                       If L0 is fixed because only one zone exists, just enter the fix L0 value.
%         rule_easting  The rule to calculate the false easting from the zone identifier ID (ell2tm).
%                       Allows following variables to be used: ID
%                       For example, in German GK it is "+5e5+ID*1e6" as 500 000 is added to avoid 
%                       negative numbers west of the central meridian and the zone ID is then put before.
%        rule_northing  The rule to calculate the false northing. [] if no false northing exists.
%                       There is no way to specify a false northing just for the southern hemisphere as
%                       it is done with UTM projection as results would become ambiguous.
%               ID_ell  The rule to calculate the zone ID when starting from ellipsoidal coordinates.
%                       Allows following variables to be used: L0
%                       Usually this derives from the L0 input, e.g. in German GK : "L0/3"
%               ID_pro  The rule to calculate the zone ID when starting from projected coordinates.
%                       Allows following variables to be used: E
%                       Usually this derives from the easting value E where the zone ID is signalled by
%                       the first numbers, e.g. in German GK: "floor(E/1e6)"
%          standard_L0  The rule of calculating the standard central meridian longitude for each point to be
%                       transformed or the longitude of origin. This is done using the longitude of the input 
%                       coordinates and is only used if L0 is omitted or empty.
%                       Allows following variables to be used: L
%          standard_B0  Latitude of Origin (=0 for the equator) 
%
% By default the following projections are definded here:
% 
% gk            German GK projection (Ellipsoid: besseldhdn)
% utm           Worldwide UTM projection without special cases - see ell2utm for details (grs80)
% bmn_gk        Austrian Bundesmeldenetz GK projection (bessel1841)
% agk28       | Austrian GK projections in M28/M31/M34 zone. 
% agk31       | AGK coordinates do not contain zone ID in easting value, so the right zone has to be
% agk34       | selected by the user.       
%
% 2. 'lambertcc2' - Conic Lambert Mapping projection
%
% The projection cell array fields are:
%
%                  lat  The first and second standard parallel [1x2] in [degrees]
%               ellips  The underlying ellipsoid as string in lower case letters
%                       The ellipsoids are stored in the mat-File "Ellipsoids.mat" also as
%                       cell-arrays named by the ellipsoid, e.g. 'bessel1841' or 'wgs84'.
%                       See "help Ellipsoids" for further details.
%                ORell  The mapped coordinate origin in ellipsoidal coordinates [LONG LAT] in [degrees]
%               ORproj  False easting and false northing of the mapped coordinate origin [E N] [meters]
%
% By default the following projection is defined here:
%
% bev           Austrian Lambert projection defined by the BEV.
% lambert93     French Lambert93 projection in RGF93 defined by the IGN.
% france_1    | French Lambert projection 
% france_2    | 
% france_2_et |
% france_3    |
% france_4    |
%
% Feel free to add your own definitions.

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Jan 18, 2006

