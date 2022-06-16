function [crackPrj] = crackProjectionMesh(crack, vert1, vert2, vert3, cameras, varargin)
%% Parse inputs
% - Set input default values
defLambda           = 10;
defWarnFlag         = true; 
defVerbose          = false;


%- Create input parse
p = inputParser;
addRequired(p,'crack',@isstruct);
addRequired(p,'vert1',@isnumeric)
addRequired(p,'vert2',@isnumeric)
addRequired(p,'vert3',@isnumeric)
addRequired(p,'cameras',@isstruct)
addOptional(p,'lambda',defLambda,@isnumeric)
addParameter(p,'warnings',defWarnFlag,@islogical)
addParameter(p,'v',defVerbose,@islogical)
parse(p, crack, vert1, vert2, vert3, cameras, varargin{:})

%- Save parsed input vals to variables
crack             = p.Results.crack;
vert1             = p.Results.vert1;
vert2             = p.Results.vert2;
vert3             = p.Results.vert3;
cameras           = p.Results.cameras;
lambda            = p.Results.lambda;
warnFlag          = p.Results.warnings;
verbose           = p.Results.v;
%% deactivate warnings
if warnFlag == false
    warning off
end

%% Projective ray - Mesh Intersection  (Triangle/Ray Intersection by Jaroslaw Tuszynski)
for cam = 1:length(crack)
    fprintf('Computing intersection for cam %i...\n', cam)
    orig           = cameras(cam).X0';                                                 % ray's origin
    sk             = ensure_homogeneous(crack(cam).skel);
    SK             = zeros(size(sk));
    left           = ensure_homogeneous(crack(cam).edgeL);
    LEFT           = zeros(size(left));
    right          = ensure_homogeneous(crack(cam).edgeR);
    RIGHT          = zeros(size(right));
    for i = 1:length(sk)
        dir = lambda * inv(cameras(cam).K * cameras(cam).R) * sk(:,i);       % ray's direction (Stachniss's formula)
        if verbose == 1; tic; end
        [intersect, ~, ~,~, xyzcoord] = TriangleRayIntersection(orig, dir, vert1, vert2, vert3, 'planeType','two sided');
        if verbose == 1
            fprintf('Cam 1 -- Number of: faces=%i, points=%i, intresections=%i; time=%f sec\n', ...
                size(triang,1), size(vertices,1), sum(intersect), toc);   
        end
        SK(:,i)     = xyzcoord(intersect==1,:)';
    end
    for i = 1:length(left)
        dir = lambda * inv(cameras(cam).K * cameras(cam).R) * left(:,i);       % ray's direction (Stachniss's formula)
        if verbose == 1; tic; end
        [intersect, ~, ~,~, xyzcoord] = TriangleRayIntersection(orig, dir, vert1, vert2, vert3, 'planeType','two sided');
        if verbose == 1
            fprintf('Cam 1 -- Number of: faces=%i, points=%i, intresections=%i; time=%f sec\n', ...
                size(triang,1), size(vertices,1), sum(intersect), toc);   
        end
        LEFT(:,i)     = xyzcoord(intersect==1,:)';
    end
    for i = 1:length(right)
        dir = lambda * inv(cameras(cam).K * cameras(cam).R) * right(:,i);       % ray's direction (Stachniss's formula)
        if verbose == 1; tic; end
        [intersect, ~, ~,~, xyzcoord] = TriangleRayIntersection(orig, dir, vert1, vert2, vert3, 'planeType','two sided');
        if verbose == 1
            fprintf('Cam 1 -- Number of: faces=%i, points=%i, intresections=%i; time=%f sec\n', ...
                size(triang,1), size(vertices,1), sum(intersect), toc);   
        end
        RIGHT(:,i)     = xyzcoord(intersect==1,:)';
    end
    crackPrj(cam).skel      = SK;
    crackPrj(cam).edgeL     = LEFT;
    crackPrj(cam).edgeR     = RIGHT;
end

end