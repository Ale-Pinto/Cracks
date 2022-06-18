function [intersection] = rayMeshIntersaction(orig, dir, mesh, varargin)
%  rayMeshIntersaction - 
% 
%   TO DO.................
% 
%  SYNTAX:
%       intersection = rayMeshIntersaction(orig, dir,mesh, varargin)
% 
%  INPUTS:
%   - orig (mandatory) --> path of the source images
%   - dir (mandatory) --> extension of the source images (without '.')
%   - mesh (mandatory) --> extension of the target images
%   - ptcloud (opt) --> extension of the target images (without '.')
%   - warnings (par) --> Allows warning outputs [default=false]
% 
%  OUTPUT:
% 
%  EXAMPLE: 
%  exifcp('img', 'jpg', '.\img\converted', 'tif', 'dh', 171.2, 'delXMP', false)
%
% To do:
%   - Add additional output parameters
% 
% Francesco Ioli 
% 25.02.2022 v1.0 

%% Check mandatory inputs
if usepar == true
    try 
       isparallel = ~isempty(ver('parallel'));
    catch
       fprintf('Parallel Computing Toolbox not installed. Using regular for loop instead.\n')
       isparallel = 0;
    end
    if isparallel == 1
        if isempty(gcp('nocreate'))
            parpool;
        end
    end
end


%% Parse inputs
%- Set default values and check functions
ptcloud         = struct();

%- Define check functions
validImgExts    = {'jpg', 'dng', 'png', 'tif', 'tiff'};
checkImgExt     = @(x) any(validatestring(x,validImgExts));

%- Create input parse
p = inputParser;
addRequired(p,'srcdirname',@ischar);
addRequired(p,'srcimgext',checkImgExt)
addRequired(p,'outdirname',defOutdirname, @ischar)
addOptional(p,'outext',defOutExt,checkImgExt)
addParameter(p,'warnings',defWarnFlag,@islogical)
parse(p,srcdirname,srcimgext,varargin{:})

%- Save parsed input vals to variables
outdirname  = p.Results.outdirname;
outExt      = p.Results.outext;
% h          = p.Results.dh;
dh          = p.Results.homeh;
delXMP      = p.Results.delXMP;
warnFlag    = p.Results.warnings;
verbose     = p.Results.v;
delOriginal = p.Results.delorig;
usepar      = p.Results.usepar;

%% deactivate warnings
if warnFlag == false
    warning off
end

%% Initialize parameters
%- Retrieve img list
fnames = dir([srcdirname filesep '*.' srcimgext]);

%- No images to process. Exit script
if size(fnames) < 0
    fprintf('No image found. Function ended.\n')
    return
end

%- Define exiftool parameters 
if verbose; vcmd = '-v'; else, vcmd = '-q -m'; end
if delOriginal; deloricmd = '-overwrite_original';else, deloricmd = ''; end


end