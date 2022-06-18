function crackPrj = crackProjectionPlane(crack, sparsePts_XYZ , A, cameras, varargin)
%% Parse inputs
% - Set input default values
defLambda           = 1;
defWarnFlag         = true; 
defVerbose          = false;
defplaneFitMaxDist  = 0.0015;
defkNearPts         = 20;

%- Create input parse
p = inputParser;
addRequired(p,'crack',@isstruct);
addRequired(p,'sparsePts_XYZ',@isnumeric)
addRequired(p,'A',@isnumeric)
addRequired(p,'cameras',@isstruct)
addOptional(p,'lambda',defLambda,@isnumeric)
addOptional(p,'planeFitMaxDist',defplaneFitMaxDist,@isnumeric)
addOptional(p,'kNearPts',defkNearPts,@isnumeric)
addParameter(p,'warnings',defWarnFlag,@islogical)
addParameter(p,'v',defVerbose,@islogical)
parse(p, crack, sparsePts_XYZ, A, cameras, varargin{:})

%- Save parsed input vals to variables
crack             = p.Results.crack;
sparsePts_XYZ     = p.Results.sparsePts_XYZ;
A                 = p.Results.A;
cameras           = p.Results.cameras;
lambda            = p.Results.lambda;
planeFitMaxDist   = p.Results.planeFitMaxDist;
kNearPts          = p.Results.kNearPts;
warnFlag          = p.Results.warnings;
verbose           = p.Results.v;

%% deactivate warnings
if warnFlag == false
    warning off
end
%% Point cloud reconstruction
sparsePC_s        = pointCloud(sparsePts_XYZ');
sparsePC_sdn      = pcdenoise(sparsePC_s);
%% Projection on the mean plane
crackPrj_t = struct();    
for cam = 1:length(crack)
    crackPrj_t(cam).skel    = rayPlaneIntersection(crack(cam).skel, cameras(cam), A, lambda);
    crackPrj_t(cam).edgeL   = rayPlaneIntersection(crack(cam).edgeL, cameras(cam), A, lambda);
    crackPrj_t(cam).edgeR   = rayPlaneIntersection(crack(cam).edgeR, cameras(cam), A, lambda);
end
%% Second projection on local planes around first projections
crackPrj = struct();
for cam = 1:length(crack)
    for tar = 1:length(crackPrj_t(cam).skel)
        fprintf('Fitting plane around target %i...', tar)
        [indices,dists] = findNearestNeighbors(sparsePC_sdn, crackPrj_t(cam).skel(:,tar)', kNearPts);
        %     [indices,dists] = findNeighborsInRadius(sparsePC_wrs,crackPrj_wrs(cam).skel(:,pt)', radius);
        locPC = select(sparsePC_sdn,indices);
        [planeFitted_s, planeFitInIdx, planeFitOutIdx, planeFitErr] = pcfitplane(locPC, planeFitMaxDist);
        fprintf('Inlier points: %i (%.0f%%); Outlier points: %i; Mean fit error: %.4f\n', ...
            length(planeFitInIdx), length(planeFitInIdx)/length(locPC.Location)*100, length(planeFitOutIdx), planeFitErr)
        Aloc       = [planeFitted_s.Parameters(1:3) -planeFitted_s.Parameters(4)]';

        crackPrj(cam).skel(:,tar)    = rayPlaneIntersection(crack(cam).skel(:,tar), cameras(cam), Aloc, lambda);
    end

    for tar = 1:length(crackPrj_t(cam).edgeL)
        fprintf('Fitting plane around target %i...', tar)
        [indices,dists] = findNearestNeighbors(sparsePC_sdn, crackPrj_t(cam).edgeL(:,tar)', kNearPts);
        %     [indices,dists] = findNeighborsInRadius(sparsePC_wrs,crackPrj_wrs(cam).skel(:,pt)', radius);
        locPC = select(sparsePC_sdn,indices);
        [planeFitted_s, planeFitInIdx, planeFitOutIdx, planeFitErr] = pcfitplane(locPC, planeFitMaxDist);
        fprintf('Inlier points: %i (%.0f%%); Outlier points: %i; Mean fit error: %.4f\n', ...
            length(planeFitInIdx), length(planeFitInIdx)/length(locPC.Location)*100, length(planeFitOutIdx), planeFitErr)
        Aloc       = [planeFitted_s.Parameters(1:3) -planeFitted_s.Parameters(4)]';

        crackPrj(cam).edgeL(:,tar)    = rayPlaneIntersection(crack(cam).edgeL(:,tar), cameras(cam), Aloc, lambda);
    end

    for tar = 1:length(crackPrj_t(cam).edgeR)
        fprintf('Fitting plane around target %i...', tar)
        [indices,dists] = findNearestNeighbors(sparsePC_sdn, crackPrj_t(cam).edgeR(:,tar)', kNearPts);
        %     [indices,dists] = findNeighborsInRadius(sparsePC_wrs,crackPrj_wrs(cam).skel(:,pt)', radius);
        locPC = select(sparsePC_sdn,indices);
        [planeFitted_s, planeFitInIdx, planeFitOutIdx, planeFitErr] = pcfitplane(locPC, planeFitMaxDist);
        fprintf('Inlier points: %i (%.0f%%); Outlier points: %i; Mean fit error: %.4f\n', ...
            length(planeFitInIdx), length(planeFitInIdx)/length(locPC.Location)*100, length(planeFitOutIdx), planeFitErr)
        Aloc       = [planeFitted_s.Parameters(1:3) -planeFitted_s.Parameters(4)]';

        crackPrj(cam).edgeR(:,tar)    = rayPlaneIntersection(crack(cam).edgeR(:,tar), cameras(cam), Aloc, lambda);
    end
end

%% Width evaluation

for kk = 1:length(crack)
    for i = 1:length(crackPrj(kk).skel)
        crackPrj(kk).width(i) = sqrt(   (crackPrj(kk).edgeR(:,i) - crackPrj(kk).edgeL(:,i))' * ...
            (crackPrj(kk).edgeR(:,i) - crackPrj(kk).edgeL(:,i)) );
    end
end

end


















