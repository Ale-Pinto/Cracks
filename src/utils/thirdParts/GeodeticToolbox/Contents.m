% Geodetic Transformations 
%
% A set of functions to calculate coordinate transformations between different reference ellipsoids
% and different projections.
%
% *** PARAMETERS AND CONSTANTS ***
%
% Ellipsoids            Definition of reference ellipsoids in Ellipsoids.mat
% Projections           Definition of projection types in Projections.mat
%                       Currently transverse mercator (TM) and Lambert conformal conical projections with
%                       two standard parallels are supported.
% Trafo_ITRF            Definition of parameters for ITRF/ETRF transformations
% Transformations       Standard transformation parameter sets for 10-parameter datum transformations,
%                       12-parameter affine transformations and 5-parameter Molodensky transformations                                    
%
% *** MAPPING AND UNMAPPING ***
%
% cart2ell              Transform ellipsoid centered cartesian coordinates to elliposidal coordinates
% ell2cart              Transform ellipsoid centered ellipsoidal coordinates to cartesian coordinates
% ell2tm                Transform ellipsoid centered ellipsoidal coordinates to any TM mapping projection
% tm2ell                Transform any TM mapping projection to ellipsoid centered ellipsoidal coordinates
% ell2utm               Transform ellipsoid centered ellipsoidal coordinates to UTM mapping projection
%                       For differences to ell2tm see "help ell2utm". 
% utm2ell               Transform UTM mapping projection to ellipsoid centered ellipsoidal coordinates
%                       For differences to tm2ell see "help utm2ell".
% lambertcc2ell         Transform any Lambert conformal conical projection (with 2 standrad parallels)
%                       to ellipsoid centered ellipsoidal coordinates
% ell2lambertcc         Transform ellipsoid centered ellipsoidal coordinates to  Lambert conformal
%                       conical projection with two standard parallels
%
% *** TRANSFORMATIONS IN CARTESIAN SYSTEMS ***
%
% d3trafo               Perform 7-parameter similarity transformation between two cartesian 3D coordinate
%                       systems to change reference ellipsoid (known as "datum transformation")
% d2trafo               Perform 4-parameter similarity transformation bewteen two cartesian 2D coordinate
%                       systems (planar transformation)
% d1trafo               Perform 2-parameter similarity transformation bewteen two cartesian 1D coordinate
%                       systems (height transformation)
% helmert3d             Deternmine transformation parameters between two cartesian 3D coordinate systems
%                       using an LSQ-adjustment on at least 3 identical points 
% helmert2d             Deternmine transformation parameters between two cartesian 2D coordinate systems
%                       using an LSQ-adjustment on at least 2 identical points
% helmert1d             Deternmine transformation parameters between two cartesian 1D coordinate systems
%                       using an LSQ-adjustment on at least 2 identical points
% d3affinetrafo         Perform 12-parameter affine transformation between two cartesian 3D coordinate
%                       systems
% d2affinetrafo         Perform 6-parameter affine transformation bewteen two cartesian 2D coordinate
%                       systems
% helmertaffine3d       Deternmine affine transformation parameters between two cartesian 3D coordinate 
%                       systems using an LSQ-adjustment on at least 4 identical points 
% helmertaffine2d       Deternmine affine transformation parameters between two cartesian 2D coordinate 
%                       systems using an LSQ-adjustment on at least 3 identical points
% d3projectivetrafo     Perform 11-parameter projective transformation between a cartesian 3D coordinate
%                       system and a cartesian 2D mapping system
% d2projectivetrafo     Perform 8-parameter projective transformation between two cartesian 2D coordinate
%                       systems
% helmertprojective3d   Deternmine projective transformation parameters between a cartesian 3D coordinate 
%                       system and a cartesian 2D mapping system using an LSQ-adjustment on at least 
%                       6 identical points 
% helmertprojective2d   Deternmine projective transformation parameters between two cartesian 2D coordinate 
%                       systems using an LSQ-adjustment on at least 4 identical points
% itrstrafo             3D-transformation between different ITRS/ETRS - realizations (frames)
%
% *** TRANSFORMATIONS IN ELLIPSOIDAL SYSTEMS ***
%
% molodenskytrafo       Perform 5-parameter Molodensky transformation between two ellipsoidal 3D coordinate
%                       systems to change reference ellipsoid (known as "datum transformation")
%                       Some assumptions on the underlying ellipsoids are used in this transformation method,
%                       please make sure that you understood the theory behind it.
%
% *** ADDITIONAL GEODETIC STUFF ***
%
% rescorr               Calculate residual correction to transformed points
% readntv2              Read a NTv2 transformation grid from ASCII file
% ntv2trafo             Perform a NTv2 grid transformation on 2D ellipsoidal coordinates
% deg2dms               Convert double degree value to degree, minute, second
% dms2deg               Convert degree, minute, second values to double degree value
%
% Type "help <function>" for details.
% See the manual in this folder for examples.
%
% Files are intended to be used in studies on Geodesy and Geoinformatics, Technical University of Munich.
% Use without any warranty.
% Additional projection types and transformations may be appended in future.
% For bug reports and questions please contact the author:
%
% Peter Wasmeier, Technical University of Munich
% p.wasmeier@bv.tum.de
% Oct-2013