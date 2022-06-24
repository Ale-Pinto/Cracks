% TRANSFORMATIONS.MAT
%
% Mat-File containing standard parameter sets for 10-parameter similarity 3D transformations  
% (datum transformations) to be used with d3trafo, for 12-parameter affine transformations used
% with d3affinetrafo or for 5-parameter Molodensky transformations.
%
% This mat-File contains parameters for different transformation types, specified by the number 
% of parameters:
%
%  2   similarity transformation 1D
%      [dz s] with
%              dz = translation [m]
%               s = scale factor
%
%  4   similarity transformation 2D
%      [dx dy ez s] with
%           dx,dy = translations [m]
%              ez = rotation [rad]
%               s = scale factor
%
%  5   Molodensky transformation
%      [dX dY dZ da df] with
%       dX,dY,dZ = translations of the ellipsoid centers [m]
%          da,df = difference of the ellipsoids semimajor axes [m] and flattening
%
%  6   affine transformation 2D
%      [x0 y0 a1 a2 a3 a4] with
%          x0,y0 = translations [m]
%       a1 to a4 = linear affine parameters
%
%  7   similarity transformation 3D Bursa Wolf type
%      [dx dy dz ex ey ez s] with
%       dx,dy,dz = translations [m]
%       ex,ey,ez = rotations in [rad]
%              s = scale factor
%       If this parameters are used, the rotation center is set to [0 0 0].
%
%  8   projective transformation 2D
%      [a1 a2 a3 a4 a5 a6 a7 a8] with
%       a1 to a8 = linear projective parameters
%
% 10   similarity transformation 3D Molodensky-Badekas type
%      [dx dy dz ex ey ez s x0 y0 z0] with
%       dx,dy,dz = translations [m]
%       ex,ey,ez = rotations in [rad]
%              s = scale factor
%       x0,y0,z0 = rotation center coordinates [m]
%
% 11   projective transformation 3D
%      [a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11] with
%      a1 to a11 = linear projective parameters
%
% 12   affine transformations 3D
%      [x0 y0 z0 a1 a2 a3 a4 a5 a6 a7 a8 a9]  with
%       x0,y0,z0 = translations [m]
%       a1 to a9 = linear affine parameters
%
% When predefined parameter sets get used, the functions needs to check whether the number of
% parameters is correct, i.e. the parameter set is qualified for the desired transformation type.
%
% Standard parameter sets mostly are used to transform WGS84 coordinates to a local datum (and back),
% but may also be used to transform local datums to each other (e.g. in border areas).
% Please note, that accuracy is limited with the use of standard parameters (usually from 1 - 25 m,
% depending on the areal extent and determination quality); especially when there is only a shift 
% information and no rotation or scaling to be appended.
% Of course, you may add your self-determined (high-precision) parameter sets into this mat-file for
% later use. See helmert3d and helmertaffine3d for details.
%
% By now the following parameter sets are defined here (all being similarity transformations):
% 
% wgs84_to_dhdn         (Potsdam datum, Germany)
% wgs84_to_s4283        (S42/83 datum, former GDR)
% wgs84_to_mgi          (Austria)
% wgs84_to_lv95         (Switzerland)
% wgs84_to_osgb36       (Great Britain)
% lb72_to_dhdn          (Belgium datum to Potsdam datum)
% rdnl_to_dhdn          (Dutch datum to Potsdam datum)
%
%
% Feel free to add your own definitions. There are many pages across the internet where standard
% parameter sets can be found, e.g. http://www.globalmapper.com/helpv9/datum_list.htm

% Author:
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Jan 18, 2006