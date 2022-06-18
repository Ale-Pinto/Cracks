function [point3d] = rayPlaneIntersection(m, cam, A, varargin)
%RAYPLANEINTERSECTION 
% 
%  SYNTAX:
%       points3d = rayPlaneIntersection(m,cam,A)
% 
%  INPUTS:
%   - m (mandatory) --> image coordinates 
%   - cam (mandatory) --> camera struct
%   - A (mandatory) --> plane parameters
%   - lambda (opt) --> 
%   - warnings (par) --> Allows warning outputs [default=true]
% 
%  OUTPUT:
% 
%  EXAMPLE: 
%       points3d = rayPlaneIntersection(m, cam, A, lambda);
%
% To do:
%   - 
% 
% Francesco Ioli 
% 22.02.2022 v1.0 


%% Check mandatory inputs


%% Parse inputs
% - Set input default values
defLambda       = 1;
defWarnFlag     = true; 
defVerbose      = false;


%- Define check functions
% validImgExts    = {'jpg', 'dng', 'png', 'tif', 'tiff'};
% checkImgExt     = @(x) any(validatestring(x,validImgExts));

%- Create input parse
p = inputParser;
addRequired(p,'m',@isnumeric);
addRequired(p,'cam',@isstruct)
addRequired(p,'A',@isnumeric)
addOptional(p,'lambda',defLambda,@isnumeric)
addParameter(p,'warnings',defWarnFlag,@islogical)
addParameter(p,'v',defVerbose,@islogical)
parse(p, m, cam, A, varargin{:})

%- Save parsed input vals to variables
m           = p.Results.m;
cam         = p.Results.cam;
A           = p.Results.A;
lambda      = p.Results.lambda;
warnFlag    = p.Results.warnings;
verbose     = p.Results.v;

%% deactivate warnings
if warnFlag == false
    warning off
end

%% Compute Ray-plane intersection

%-  
skew            = [0 -A(3) A(2); A(3) 0 -A(1); -A(2) A(1) 0];
wedge           = [A(4)* diag([1 1 1]), -skew; -A(1:3)', [0 0 0]];

% %- Make c opposite
% rot180degX = [-1 0 0; 0 -1 0; 0 0 1];
% cam.K      = cam.K * rot180degX;
% cam.R      = cam.R * rot180degX;
% cam.P      = cam.K * [cam.R cam.t];
% cam.X0     = - cam.P(:,1:3)\cam.P(:,4);

%- Initialize variables 
point3d         = zeros(3,size(m,2));

%- Big Loop
for i = 1:size(m,2)
    point_inf   = [ inv(cam.K * cam.R) * m(:,i); 0];
    point       = [ - cam.X0 + inv(cam.K * cam.R) * m(:,i); lambda]; 
%     point_inf   = [ -inv(cam.K*cam.R) * cam.K*cam.t  + inv(cam.K*cam.R) * m(:,i); 0];
%     point       = [ -inv(cam.K*cam.R) * cam.K*cam.t  + inv(cam.K*cam.R) * m(:,i); lambda]; 
    linea_pluc  = [ point_inf(4)*[point(1:3)] - point(4)*point_inf(1:3); cross(point_inf(1:3), point(1:3))];
    point_hom   = wedge * linea_pluc;
    point3d(:,i) = - point_hom(1:3)/point_hom(4);

end

% point_inf = [ cameras(1).X0 + KR1 * crack(1).edgeR(:,i); 0];
% point = [cameras(1).X0 + KR1 * crack(1).edgeR(:,i); lambda]; 
% linea_pluc = [point_inf(4)*[point(1:3)] - point(4)*point_inf(1:3); cross(point_inf(1:3),point(1:3))];
% point_hom = wedge*linea_pluc;
% point3d1_right(:,i) = -[point_hom(1:3)/point_hom(4)];

end

