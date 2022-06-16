%%
clear; clc;
rootDirPath = ('V:\2022\crepePonti\ponteTuna_2022_01_26\Fessure Matlab\crepeStereo');
cd(rootDirPath);

%-
imFld               = 'img'; 
imExt               = '.tif';
calibFld            = 'calib';
% GCPWorldCoordFlnm   = 'marker_crepa_gcp.csv';                   % File with GCP world coordinates
% CPWorldCoordFlnm    = 'marker_crepa_cp.csv';                    % File with CP world coordinates
% ImageCoordFlnm      = 'crepe_targets_imageCoordinates.mat';     % File with already collimate image targets coordinate
ImageOneCrack       = 'DJI_0832.mat';                           % Crack image coordinates already detected in first image and saves as struct
ImageTwoCrack       = 'DJI_0835.mat';                           % Crack image coordinates already detected in second image and saves as struct
meshFlnm            = 'mesh_prova.ply';            % File for the scene mesh from metashape
denseCloud          = 'PC_densa.las';                  % File with dense metashape point cloud

%- Optional passages 
undistimgs      = true;      % Apply image distortions
enhanceimgs     = true;      % Enhanche images before elaborations
printFigs       = false;      % Print figures during elaborations

%- Chose between working in sLCRS or in WRS
%- sLCRS (scaled Local Camera Reference System) --> onlyScale == 1
%- WRS: (World Reference System) --> onlyScale == 0
onlyScale       = true;

%- Execute the algorithm to find crack edges and skeleton 
findCrack       = true;

%- Get target coordinates in images 
getTarget       = false;     % If targets or control points availabe

noMarker        = true;      % No markers in the scene, use distances betweeen cameras

%- Choose between fitting a plane or use a mesh (fitPlane = 0 --> mesh, fitPlane = 1 --> plane)
fitPlane        = false;

%% Add path to toolboxes
addpath(genpath(fullfile(rootDirPath,'\mat')));

%- Run VLfeat setup
vl_setup;

%% Load data
%- Get lists of images and calibration data
images  = imageDatastore(fullfile(rootDirPath, imFld),'FileExtensions',{imExt}); 
calibfln = dir(fullfile(rootDirPath, calibFld,'*.mat'));

%- Read images
fprintf('Reading images... ')
I1      = readimage(images, 1);
I2      = readimage(images, 2);
fprintf(' Done.\n')    

%- load camera params
fprintf('Reading calibration data... ')
calib = struct();
for i = 1:length(calibfln)
    tmp = load(fullfile(calibfln(i).folder,calibfln(i).name));
    calib(i).cameraParams = tmp.cameraParams;
    if isfield(tmp, 'estimationErrors')
        calib(i).estimationErrors = tmp.estimationErrors;
    end
end
fprintf(' Done.\n')
clearvars tmp

%- If already extracted skeleton and edges: change internal variable names
%  manually. Undistortion can be done or not.
if findCrack == 0
   fprintf('Reading crack data... ')
   load(ImageOneCrack)
   load(ImageTwoCrack)
   un1 = ones(length(s0832),1)';
   un2 = ones(length(s0835),1)';
   crack = struct();
   crack(1).skel  = [undistortPoints([s0832.X_skel; s0832.Y_skel]',calib.cameraParams)'; un1];
   crack(2).skel  = [undistortPoints([s0835.X_skel; s0835.Y_skel]',calib.cameraParams)'; un2];
   crack(1).edgeL = [undistortPoints([s0832.X_edge_s; s0832.Y_edge_s]',calib.cameraParams)'; un1];
   crack(2).edgeL = [undistortPoints([s0835.X_edge_s; s0835.Y_edge_s]',calib.cameraParams)'; un2];
   crack(1).edgeR = [undistortPoints([s0832.X_edge_d; s0832.Y_edge_d]',calib.cameraParams)'; un1];
   crack(2).edgeR = [undistortPoints([s0835.X_edge_d; s0835.Y_edge_d]',calib.cameraParams)'; un2];
   clearvars s1 s2 un1 un2
   fprintf(' Done.\n')
end

%- Load image coordinates of the targets if already have them
if getTarget == 0 && noMarker ==  0
   fprintf('Reading target coordinates... ')
   load(ImageCoordFlnm)
   %- Read targets GCP coordinate
   gcpXYZtab   = readXYZpts(GCPWorldCoordFlnm);
   gcpXYZ      = [gcpXYZtab.E gcpXYZtab.N gcpXYZtab.h];
   gcp(1).ms60      = gcpXYZ';
   gcp(2).ms60      = gcpXYZ';
   %- Read CP world coordinate
   cpXYZtab   = readXYZpts(CPWorldCoordFlnm);
   cpXYZ      = [cpXYZtab.E cpXYZtab.N cpXYZtab.h];
   cp(1).ms60    = cpXYZ';
   cp(2).ms60    = cpXYZ';
   fprintf(' Done.\n')
end

% %% Read targets coordinates (sistemare struttura dati import!)
% %- Read GCPs targets image and world coordinates (or collimate targets on images)
if getTarget == 1
    fprintf('Collimate GCP... ')
    %- Read GCPs world coordinate  
    gcpXYZtab   = readXYZpts(GCPWorldCoordFlnm);
    gcpXYZ      = [gcpXYZtab.E gcpXYZtab.N gcpXYZtab.h];

    %- GCPs collimation on the images
    if (~exist('pi1','var') && exist('pi2','var'))
        [pi1,pi2] = cpselect(I1,I2,'Wait',true);
    else
        [pi1, pi2] =  cpselect(I1,I2,pi1,pi2,'Wait',true);
    end
    gcp              = struct();
    gcp(1).im         = pi1';
    gcp(2).im         = pi2';
    gcp(1).ms60      = gcpXYZ';
    gcp(2).ms60      = gcpXYZ';
    clearvars pi1 pi2 gcpXYZ

%- Read CP targets image coordinates (or collimate targets) (SISTEMARE    %STRUTTURA DATI CP!!!!)  --------------%
    %- Read CPs world coordinate
    cpXYZtab   = readXYZpts(CPWorldCoordFlnm);
    cpXYZ      = [cpXYZtab.E cpXYZtab.N cpXYZtab.h];

    %- CPs collimation on the images
    fprintf('Collimate CP... ')
    if (~exist('cp1','var') && exist('cp2','var'))
        [cp1,cp2] = cpselect(I1,I2,'Wait',true);
    else
        [cp1, cp2] =  cpselect(I1,I2,cp1,cp2,'Wait',true);
    end
    cp            = struct();
    cp(1).im       = cp1';
    cp(2).im       = cp2';
    cp(1).ms60    = cpXYZ';
    cp(2).ms60    = cpXYZ';
    clearvars cp1 cp2 cpXYZ
    fprintf(' Done.\n')
end
fprintf(' Done.\n')
%% Image pre processing
%- Enhance images
if enhanceimgs == 1
    fprintf('Enhancing images... ')
    LAB = rgb2lab(I1);
    L = LAB(:,:,1)/100;
    L = adapthisteq(L,'NumTiles',[8 8],'ClipLimit',0.005);
    L = imsharpen(L, 'Radius', 1, 'Amount', 1, 'Threshold', 0);
    LAB(:,:,1) = L*100;
    I1 = lab2rgb(LAB);

    LAB = rgb2lab(I2);
    L = LAB(:,:,1)/100;
    L = adapthisteq(L,'NumTiles',[8 8],'ClipLimit',0.005);
    L = imsharpen(L, 'Radius', 1, 'Amount', 1, 'Threshold', 0);
    LAB(:,:,1) = L*100;
    I2 = lab2rgb(LAB);
    
    fprintf('Done.\n')    
end

%- Undistort images
if undistimgs == 1
    fprintf('Undistorting images... ')    
    I1 = undistortImage(I1, calib.cameraParams);
    I2 = undistortImage(I2, calib.cameraParams);
    if printFigs == 1
        figure('Name','undistortedImgs') 
        imshowpair(I1, I2, 'montage');
        title('Undistorted Images');
    end
    fprintf('Done.\n')        
end  
%% Run edge and skeleton detection
if findCrack == 1
    s1 = detectCrack(I1);
%     s2 = detectCrack(I2);
    un1 = ones(length(s1),1)';
%     un2 = ones(length(s2),1)';
    crack = struct();
    crack(1).skel = [s1.X_skel; s1.Y_skel; un1];
%     crack(2).skel = [s2.X_skel; s2.Y_skel; un2];
    crack(1).edgeL = [s1.X_edge_s; s1.Y_edge_s; un1];
%     crack(2).edgeL = [s2.X_edge_s; s2.Y_edge_s; un2];
    crack(1).edgeR = [s1.X_edge_d; s1.Y_edge_d; un1];
%     crack(2).edgeR = [s2.X_edge_d; s2.Y_edge_d; un2];
%     clearvars s1 s2 un1 un2
end
%% Define Local Cam1 Reference System (LCRS) and Define Camera Structures 
cameras = struct();

%- Intrinsics matrix
K               = calib.cameraParams.IntrinsicMatrix';

%- Define Camera 1 structure
cameras(1).K    = K;   
cameras(1).t    = zeros(3,1);     % Fix Camera 1 as origin of LCRS (R=I)
cameras(1).R    = eye(3,3);
cameras(1).P    = cameras(1).K * [cameras(1).R cameras(1).t];
cameras(1).X0   = - cameras(1).P(:,1:3)\cameras(1).P(:,4);

%- Define Camera 2 structure
cameras(2).K    = K;
%% Estimate Camera Relative Orientation

%- Define Matches structue
dpts   = struct(); 

%- Run VL_SIFT
fprintf('Run VL_SIFT to find omologous points: ...')
Ia = single(rgb2gray(I1));
Ib = single(rgb2gray(I2));

siftpar.peak_thresh = 0.5; % 0.025
siftpar.edge_thresh = 10; % 10
siftpar.windowSize  = 3;
siftpar.levels      = 3;
siftpar.magnif      = 4;
[dpts(1).f, dpts(1).d] = vl_sift(Ia, ...
    'Levels', siftpar.levels, 'WindowSize',siftpar.windowSize, 'edgethresh', siftpar.edge_thresh, ...
    'PeakThresh', siftpar.peak_thresh, 'Magnif', siftpar.magnif); %, 'verbose');
[dpts(2).f, dpts(2).d] = vl_sift(Ib, ...
    'Levels', siftpar.levels, 'WindowSize',siftpar.windowSize,'edgethresh', siftpar.edge_thresh, ...
    'PeakThresh', siftpar.peak_thresh, 'Magnif', siftpar.magnif); %, 'verbose');
% fa = getfield(dpts(1), 'f');
% fb = getfield(dpts(2), 'f');
% figure, imshow(uint8(Ia)), hold on;
% h1 = vl_plotframe(fa);
% h2 = vl_plotsiftdescriptor(da,fa);
% figure, imshow(uint8(Ib)), hold on;
% h1 = vl_plotframe(fb);
clearvars Ia Ib
fprintf('Done.\n') 

%- Find putative matches 
fprintf('Find putative matches.... ') 
matchThreshol = 1.5; % 2.0
[matches, scores] = vl_ubcmatch(dpts(1).d,dpts(2).d,matchThreshol) ;
dpts(1).pts = dpts(1).f(1:2,matches(1,:));
dpts(2).pts = dpts(2).f(1:2,matches(2,:));

%- View matches 
% figure('Name', 'Matched point'), hold on;
% imshowpair(Ia,Ib, 'montage');
% vl_plotframe(fa(:,matches(1,:))) ;
% fb_plt = fb; fb_plt(1,:) = fb(1,:) + size(Ia,2) ;
% vl_plotframe(fb_plt(:,matches(2,:))) ;
% h = line([pts1(1,:) ; pts2(1,:)+size(Ia,2)], [pts1(2,:) ; pts2(2,:)]); set(h,'linewidth', 0.5, 'color', 'b') ;

fprintf('Done.\n') 

%- Estimate Essential Matrix (with E-estim toolbox)
fprintf('Estimating Essential Matrix... ') 
eMatDistanceTresh  = 0.00001;
dpts(1).npts   = cameras(1).K \ ensure_homogeneous(dpts(1).pts);            % [dpts(1).pts; ones(1, size(dpts(1).pts,2))];
dpts(2).npts   = cameras(2).K \ ensure_homogeneous(dpts(2).pts);            % [dpts(2).pts; ones(1, size(dpts(2).pts,2))];
[E, inliersIdx] = ransacfitessmatrix(dpts(1).npts, dpts(2).npts, eMatDistanceTresh);
dpts(1).inlierPts = dpts(1).pts(:,inliersIdx);
dpts(2).inlierPts = dpts(2).pts(:,inliersIdx);
fprintf('Inliers: %.0f%%\n',length(inliersIdx)/length(dpts(1).npts)*100)

%- View matches after epipolar rejection
I1corners = cornerPoints(dpts(1).inlierPts');
I2corners = cornerPoints(dpts(2).inlierPts');

if printFigs == 1
    figure;
    showMatchedFeatures(I1, I2, I1corners, I2corners,'montage');
    legend('Inlier points in I1', 'Inlier points in I2')
    clearvars I1corners I2corners
    fprintf('done.\n')
end

%- Estimate camera 2 EO
fprintf('Estimating Camera 2 EO in LCRS... ') 
[cameras(2).R, cameras(2).t] = relative_lin(dpts(2).inlierPts, dpts(1).inlierPts, ...
        cameras(2).K, cameras(1).K);
[cameras(2).R, cameras(2).t] = relative_nonlin(cameras(2).R, cameras(2).t, ...
    dpts(2).inlierPts, dpts(1).inlierPts, cameras(2).K, cameras(1).K);
cameras(2).P      = cameras(2).K * [cameras(2).R cameras(2).t'];
cameras(2).X0     = - cameras(2).P(:,1:3)\cameras(2).P(:,4);

fprintf('done.\n')
%% Sparse point cloud triangulation

%- Tringulation 
fprintf('Triangulating omologous points: ...')
sparsePts_XYZ = triang_lin_batch( {cameras(1).P, cameras(2).P}, {dpts(1).inlierPts, dpts(2).inlierPts} );
for i = 1:size(sparsePts_XYZ,2)
    sparsePts_XYZ(:, i) = triang_nonlin(sparsePts_XYZ(:,i), ...
         {cameras(1).P, cameras(2).P}, {dpts(1).inlierPts(:,i), dpts(2).inlierPts(:,i)});
end
fprintf('done.\n')

% %- Interpolate points color on I1
% fprintf('Interpolating colors on image 1: ...')
% col = interpPointCol(sparsePts_XYZ, I1, cameras(1).P);

if printFigs == 1
    cameraSize = 0.2;
    sparsePC    = pointCloud(sparsePts_XYZ');
    sparsePC_dn = pcdenoise(sparsePC);
    figure; hold on;
    pcshow(sparsePC_dn, 'VerticalAxis','Y', 'VerticalAxisDir','Down')
    xlabel('X'), ylabel('Y'), zlabel('Z')
    plotCamera('Location', cameras(1).X0, 'Orientation', cameras(1).R, 'Size', cameraSize, ...
        'Color', [0.9294 0.6941 0.1255], 'Opacity', 0);
    plotCamera('Location', cameras(2).X0, 'Orientation', cameras(2).R, 'Size', cameraSize, ...
        'Color', [0.3020 0.7451 0.9333], 'Opacity', 0);
    line([0 0.3], [0 0], [0 0], 'Color', 'r', 'LineWidth',2)
    line([0 0], [0 0.3], [0 0], 'Color', 'g', 'LineWidth',2)
    line([0 0], [0 0], [0 0.3], 'Color', 'b', 'LineWidth',2)
end 
fprintf('done.\n')

%% Scale model by using one or n (least-squares) distances ----> IMPLEMENT LEAST SQUARES!!!
if onlyScale == 1

    fprintf('Working in sLCRS. Scaling the model with reference distances... \n')

    %- Estimate targets coordinates in LCRS by tringulation 
    if getTarget == 1
        gcp(1).triang = triang_lin_batch( {cameras(1).P, cameras(2).P}, {gcp(1).im, gcp(2).im} );
        for i = 1:size(gcp(1).triang ,2)
            gcp(1).triang (:, i) = triang_nonlin(gcp(1).triang(:,i), ...
                {cameras(1).P, cameras(2).P}, {gcp(1).im(:,i), gcp(2).im(:,i)});
        end
        gcp(2).triang = gcp(1).triang;

        %- Estimate scale factor
        idx = [1,4; ...
            ];                                                       %- Use target 101 (row 1) and 112 (row 4)
        d0 = zeros(size(idx,1),1); D0 = zeros(size(idx,1),1);
        for i = 1:size(idx,1)
            d0(i) = sqrt(   ( gcp(1).triang(:,idx(i,1)) - gcp(1).triang(:,idx(i,2)) )' *  ...     %- Estimate distance between targets in LCRS (not scaled)
                ( gcp(1).triang(:,idx(i,1)) - gcp(1).triang(:,idx(i,2)) ) );
            D0(i) = sqrt(   ( gcp(1).ms60(:,idx(i,1))   - gcp(1).ms60(:,idx(i,2)) )'  * ...        %- Estimate distance between targets in WRS (scaled)
                ( gcp(1).ms60(:,idx(i,1))   - gcp(1).ms60(:,idx(i,2)) ) );
            scaleFct = D0 ./ d0;
        end
    end

    if noMarker ==1
        X0 = cameras(2).X0;
        dist_im = sqrt((0 - X0(1))^2 + (0 - X0(2))^2 + (0 - X0(3))^2);                                                 % Estimate distance between cameras in LCRS
        dist_real = sqrt((997.150467 - 998.565244)^2 + (1017.608218 - 1017.264502)^2 + (104.634822 - 104.593230)^2);    % Estimate distance between cameras in WRS
        scaleFct = dist_real/dist_im; % metri
    end

    fprintf('Estimated scale factor: %.5f\n', scaleFct)

    %- Scale Sparse Point Cloud and cam2 EO
    sparsePts_XYZ        = sparsePts_XYZ * scaleFct;
    cameras(2).X0        = cameras(2).X0 * scaleFct;
    cameras(2).t         = - cameras(2).R * cameras(2).X0;
    cameras(2).P         = cameras(2).K * [cameras(2).R cameras(2).t];

    if getTarget == 1
        gcp(1).triang        = gcp(1).triang * scaleFct;
    end

    %- Create again and plot Sparse Point Cloud with Matlab toolbox
    if printFigs == 1
        sparsePC        = pointCloud(sparsePts_XYZ');
        sparsePC_dns    = pcdenoise(sparsePC);
        figure; hold on;
        cameraSize = 0.2;    
        pcshow(sparsePC_dns, 'VerticalAxis','Y', 'VerticalAxisDir','Down', 'MarkerSize',45)
        xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')
        plotCamera('Location', cameras(1).X0, 'Orientation', cameras(1).R, 'Size', cameraSize, ...
            'Color', 'b', 'Label', '1', 'Opacity', 0);
        plotCamera('Location', cameras(2).X0, 'Orientation', cameras(2).R, 'Size', cameraSize, ...
            'Color', 'r', 'Label', '2', 'Opacity', 0);
    end

    if getTarget == 1
        cp(1).triang = triang_lin_batch({cameras(1).P cameras(2).P}, {cp(1).im cp(2).im});
        for i = 1:size(cp(1).triang ,2)
            cp(1).triang(:, i) = triang_nonlin(cp(1).M (:,i), ...
                {cameras(1).P cameras(2).P}, {cp(1).im(:,i) cp(2).im(:,i)});
        end
        cp(2).triang = cp(1).triang;
    end

% %% Run Dense SIFT --> optiponal
% fprintf('Find putative matches.... ') 
% 
% [I1cut,rect] = imcrop(im2single(rgb2gray(I1)));
% [I2cut] = imcrop(im2single(rgb2gray(I2)),rect);
% 
% [f1, d1] = vl_phow(I1cut, 'Step', 16) ;
% [f2, d2] = vl_phow(I2cut, 'Step', 16) ;
% 
% % image(I1); hold on
% % perm = randperm(size(f1,2)) ;
% % sel = perm(1:10000) ;
% % h1 = vl_plotframe(f1(:,sel)) ;
% % h2 = vl_plotframe(f1(:,sel)) ;
% % set(h1,'color','k','linewidth',2) ;
% % set(h2,'color','y','linewidth',1) ;
% 
% matchThreshol = 1.5;
% [matches, scores] = vl_ubcmatch(d1,d2,matchThreshol) ;
% 
% dpts(1).pts = dpts(1).f(1:2,matches(1,:));
% dpts(2).pts = dpts(2).f(1:2,matches(2,:));

%% FIT Plane on sparse cloud (in LCRS - scaled)

    if fitPlane == 1

        %- Fit Plane
        fprintf('Fitting plane to point cloud data... ')
        planeFitMaxDist = 0.005;
        [planeFitted, planeFitInIdx, planeFitOutIdx, planeFitErr] = pcfitplane(sparsePC_dns, planeFitMaxDist);
        fprintf('Plane fitted. \nInlier points: %i (%.0f%%); Outlier points: %i; Mean fit error: %.4f\n', ...
            length(planeFitInIdx), length(planeFitInIdx)/length(sparsePts_XYZ)*100, length(planeFitOutIdx), planeFitErr)
%         clearvars pc
        fprintf('Done.\n ')

        %- Plot Sparse Point Cloud and fitted plane
        if printFigs == 1
            figure; hold on;
            pcshow(sparsePC_dns, 'VerticalAxis','Y', 'VerticalAxisDir','Down', 'MarkerSize',45)
            xlabel('x'), ylabel('y'), zlabel('z')
            plotCamera('Location', cameras(1).X0, 'Orientation', cameras(1).R, 'Size', cameraSize, ...
                'Color', 'b', 'Label', '1', 'Opacity', 0);
            plotCamera('Location', cameras(2).X0, 'Orientation', cameras(2).R, 'Size', cameraSize, ...
                'Color', 'r', 'Label', '2', 'Opacity', 0);
            plot(planeFitted, 'Color', [0.7 0.7 0.7])
        end
%% Project crack image points on fitted plane LCRS (Plucker's method)

        fprintf('Projecting crack on the fitted plane... ')

        A = [planeFitted.Parameters(1:3) -planeFitted.Parameters(4)]';
        crackPrj = crackProjectionPlane(crack, sparsePts_XYZ , A, cameras);

        fprintf('Done. \n')

   %- Plots
        if printFigs == 1
            %- Camera 1 projection
            null = zeros(length(crackPrj(1).width'),1);
            skel1PC_p = pointCloud(crackPrj(1).skel', 'Color',[100*crackPrj(1).width' null null]); %, , 'Intensity',crackPrj(1).width', 'Color', repmat([1 0 0], [length(crackPrj(1).skel'), 1])
            figure; hold on;
            grid on
            pcshow(skel1PC_p, 'VerticalAxis','Y', 'VerticalAxisDir','Down', 'MarkerSize',45)
            xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')
            plot(planeFitted, 'Color', [0.7 0.7 0.7])
            plot3(crackPrj(1).edgeR(1,:),crackPrj(1).edgeR(2,:),crackPrj(1).edgeR(3,:), '.g', 'MarkerSize', 10)
            plot3(crackPrj(1).edgeL(1,:),crackPrj(1).edgeL(2,:),crackPrj(1).edgeL(3,:), '.g', 'MarkerSize', 10)
            title('Crack projection from first image')

            %- Camera 2 projection
            null_2 = zeros(length(crackPrj(2).width'),1);
            skel2PC_p = pointCloud(crackPrj(2).skel', 'Color',[100*crackPrj(2).width' null null]); %, , 'Intensity',crackPrj(1).width', 'Color', repmat([1 0 0], [length(crackPrj(1).skel'), 1])
            figure; hold on;
            grid on
            pcshow(skel2PC_p, 'VerticalAxis','Y', 'VerticalAxisDir','Down', 'MarkerSize',45)
            xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')
            plot(planeFitted, 'Color', [0.7 0.7 0.7])
            plot3(crackPrj(2).edgeR(1,:),crackPrj(2).edgeR(2,:),crackPrj(2).edgeR(3,:), '.g', 'MarkerSize', 10)
            plot3(crackPrj(2).edgeL(1,:),crackPrj(2).edgeL(2,:),crackPrj(2).edgeL(3,:), '.g', 'MarkerSize', 10)
            title('Crack projection from second image')
            clearvars null null_2
        end
    end

    if fitPlane == 0
 %- Read PLY mesh

        fprintf('Reading Metashape Mesh... ')

        [vertices, triang] = readMesh_ply(meshFlnm);
        vert1 = vertices(triang(:,1),:);
        vert2 = vertices(triang(:,2),:);
        vert3 = vertices(triang(:,3),:);

        % meshVertPC = pointCloud(vertices);
        if printFigs == 1
            figure('Name', 'Mesh from Metashape'); hold on
            title('Sparse Point Cloud Triangulation','fontsize',10)
            axis equal
            trisurf(triang,vertices(:,1),vertices(:,2),vertices(:,3),'facecolor','c','edgecolor','b') %plot della superficie
            axis vis3d
            view(3)
        end
        fprintf('done.\n')

    %- Read Metashape Dense Cloud
        fprintf('Reading Metashape Dense Cloud... ')

%         densePC = pointCloud(denseCloud);
densePC = pcread('pc_densa.ply');
        if printFigs == 1
            densePC.plot('Color', 'A.rgb','MarkerSize', 2); hold on
            xlabel('x'), ylabel('y'), zlabel('z')
            plotCamera('Location', X01_BBA, 'Orientation', R1_bba, 'Size', cameraSize, ...
                'Color', 'b', 'Label', '1', 'Opacity', 0);
            plotCamera('Location', X02_BBA, 'Orientation', R2_bba, 'Size', cameraSize, ...
                'Color', 'r', 'Label', '2', 'Opacity', 0);
        end
        fprintf('done.\n')
%% Projective ray - Mesh Intersection 

        fprintf('Projecting crack on the mesh... ')

        crackPrj = crackProjectionMesh(crack, vert1, vert2, vert3, cameras);

        fprintf('Done.\n')

   %- Plots
        if printFigs == 1
            %- Camera 1 projection
            figure; hold on;
            densePC.plot('Color', 'A.rgb','MarkerSize', 2); hold on
            xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')
%             pcshow(sparsePC, 'VerticalAxis','Z', 'VerticalAxisDir','Up', 'MarkerSize',45)
            plot3(crackPrj(1).skel(1,:),crackPrj(1).skel(2,:),crackPrj(1).skel(3,:), '.r', 'MarkerSize',5)
            plot3(crackPrj(1).edgeL(1,:),crackPrj(1).edgeL(2,:),crackPrj(1).edgeL(3,:), '.g', 'MarkerSize',5)
            plot3(crackPrj(1).edgeR(1,:),crackPrj(1).edgeR(2,:),crackPrj(1).edgeR(3,:), '.g', 'MarkerSize',5)

            %- Camera 2 projection
            figure; hold on;
            densePC.plot('Color', 'A.rgb','MarkerSize', 2); hold on
            xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')
%             pcshow(sparsePC, 'VerticalAxis','Z', 'VerticalAxisDir','Up', 'MarkerSize',45)
            plot3(crackPrj(2).skel(1,:),crackPrj(2).skel(2,:),crackPrj(2).skel(3,:), '.r', 'MarkerSize',5)
            plot3(crackPrj(2).edgeL(1,:),crackPrj(2).edgeL(2,:),crackPrj(2).edgeL(3,:), '.g', 'MarkerSize',5)
            plot3(crackPrj(2).edgeR(1,:),crackPrj(2).edgeR(2,:),crackPrj(2).edgeR(3,:), '.g', 'MarkerSize',5)
        end
    end
   %% Export pointcloud
    pcwrite(pointCloud(crackPrj(1).skel', 'Intensity',crackPrj(1).width'), fullfile('pcOut','skel1.ply'));
    pcwrite(pointCloud(crackPrj(2).skel', 'Intensity',crackPrj(2).width'), fullfile('pcOut','skel2.ply'));
    pcwrite(pointCloud(crackPrj(1).edgeL'), fullfile('pcOut','left1.ply'));
    pcwrite(pointCloud(crackPrj(1).edgeR'), fullfile('pcOut','rigth1.ply'));
    pcwrite(pointCloud(crackPrj(2).edgeL'), fullfile('pcOut','left2.ply'));
    pcwrite(pointCloud(crackPrj(2).edgeR'), fullfile('pcOut','rigth2.ply'));

    fprintf('End. \n\n')
end
%% Estimate rototraslation of LCRS to WRS (Helmert 3d transformation by GeodeticToolbox)

if onlyScale == 0
    fprintf('Working in WRS. Estimating Helmert3D transformation... \n')

    %- Estimate Helmert3D transformation
    [helmAff3dTp,~,~]      = helmertaffine3d(gcp(1).triang', gcp(1).ms60'); 
    [helm3dTp,~,trasfAccu,transfRes] = helmert3d(gcp(1).triang', gcp(1).ms60' ,'7p', 0, helmAff3dTp(4:6)); 
    fprintf('Estimated Helmert Transformation:\n')
    fprintf('T :\t\t [%.4f %.4f %.4f] [m],\t Accuracy: [%.4f %.4f %.4f] [m]\n', helm3dTp(1:3),trasfAccu(1:3))
    fprintf('R :\t\t [%.4f %.4f %.4f] [rad],\t\t\t Accuracy: [%.4f %.4f %.4f] [rad]\n', helm3dTp(4:6),trasfAccu(4:6))
    fprintf('scale :\t %.6f [/], \t\t\t\t\t\t\t Accuracy: %.6f [/]\n', helm3dTp(7),trasfAccu(7))
    fprintf('Transformation residuals:\n')
    fprintf('X [m]\t\t Y [m]\t\t Z [m]\n')
    for i = 1:size(transfRes,1)
        fprintf('%+5.4f\t\t %+5.4f\t %+5.4f\n', transfRes(i,:))
    end 
    fprintf('Done. \n')
    
    % - Apply Helmert3D transformation to cameras
    R2wrs               = buildRotMat(helm3dTp(4:6));
    cameras_wrs         = struct();
    cameras_wrs(1).K    = K;   
    cameras_wrs(2).K    = K;
    for i = 1:length(cameras)
        cameras_wrs(i).X0   = d3trafo(cameras(i).X0', helm3dTp)';
        cameras_wrs(i).R    = cameras(i).R * R2wrs';
        cameras_wrs(i).t    = -cameras_wrs(i).R * cameras_wrs(i).X0;
        cameras_wrs(i).P    = cameras_wrs(i).K * [cameras_wrs(i).R cameras_wrs(i).t];
    end

    % - Apply Helmert3D transformation to point cloud
    XYZ_wrs         = d3trafo(sparsePts_XYZ, helm3dTp);

    %- Plots
    if printFigs == 1
        sparsePC_wrs    = pointCloud(XYZ_wrs);
        sparsePC_wrs    = pcdenoise(sparsePC_wrs);
        figure; hold on;
        pcshow(sparsePC_wrs, 'VerticalAxis','Z', 'VerticalAxisDir','Up', 'MarkerSize',45)
        xlabel('x'), ylabel('y'), zlabel('z')
        plotCamera('Location', cameras_wrs(1).X0, 'Orientation', cameras_wrs(1).R, 'Size', cameraSize, ...
            'Color', 'b', 'Label', '1', 'Opacity', 0);
        plotCamera('Location', cameras_wrs(2).X0, 'Orientation', cameras_wrs(2).R, 'Size', cameraSize, ...
            'Color', 'r', 'Label', '2', 'Opacity', 0);
    end

    %- Estimate GCPs and CPs targets coordinates in WRS by tringulation 
    gcp(1).wrs = triang_lin_batch({cameras_wrs(1).P cameras_wrs(2).P}, {gcp(1).im gcp(2).im});
    for i = 1:size(gcp(1).wrs,2)
        gcp(1).wrs(:, i) = triang_nonlin(gcp(1).wrs (:,i), ...
            {cameras_wrs(1).P cameras_wrs(2).P}, {gcp(1).im(:,i) gcp(2).im(:,i)});
    end
    gcp(2).wrs = gcp(1).wrs;

    cp(1).wrs = triang_lin_batch({cameras_wrs(1).P cameras_wrs(2).P}, {cp(1).im cp(2).im});
    for i = 1:size(cp(1).wrs ,2)
        cp(1).wrs(:, i) = triang_nonlin(cp(1).wrs (:,i), ...
            {cameras_wrs(1).P cameras_wrs(2).P}, {cp(1).im(:,i) cp(2).im(:,i)});
    end
    cp(2).wrs = cp(1).wrs;
    
    %- Evaluate errors on CPs
%     errors1 = cp_wrs(1).M - cp_wrs(1).ms60;
%     errorsRMSE = sqrt(mean(errors1.^2));
%     fprintf('World points error RMSE on CPs [px]\n')
%     fprintf('X: %.4f [m]\n', errorsRMSE(1))
%     fprintf('Y: %.4f [m]\n', errorsRMSE(2))
%     fprintf('Z: %.4f [m]\n', errorsRMSE(3)) 
%% FIT Plane on sparse cloud (in WRS)

    if fitPlane == 1
        fprintf('Fitting plane to point cloud data... ')
        planeFitMaxDist = 0.005;
        [planeFitted_WRS, planeFitInIdx_WRS, planeFitOutIdx_WRS, planeFitErr_WRS] = pcfitplane(sparsePC_wrs, planeFitMaxDist);
        fprintf('Plane fitted. \nInlier points: %i (%.0f%%); Outlier points: %i; Mean fit error: %.4f\n', ...
            length(planeFitInIdx_WRS), length(planeFitInIdx_WRS)/length(sparsePts_XYZ)*100, length(planeFitOutIdx_WRS), planeFitErr_WRS)
        fprintf('Done.\n ')

    %- Plot Sparse Point Cloud and fitted plane
        if printFigs == 1
            figure; hold on;
            pcshow(sparsePC_wrs,'MarkerSize',45)
            xlabel('x'), ylabel('y'), zlabel('z')
            plotCamera('Location', cameras_wrs(1).X0, 'Orientation', cameras_wrs(1).R, 'Size', cameraSize, ...
                'Color', 'b', 'Label', '1', 'Opacity', 0);
            plotCamera('Location', cameras_wrs(2).X0, 'Orientation', cameras_wrs(2).R, 'Size', cameraSize, ...
                'Color', 'r', 'Label', '2', 'Opacity', 0);
            plot(planeFitted_WRS, 'Color', [0.7 0.7 0.7])
        end
%% Crack skeleton and edge in WRS

        fprintf('Projecting crack on the plane... ')

        A_wrs = [planeFitted_WRS.Parameters(1:3) -planeFitted_WRS.Parameters(4)]';
        crackPrj_wrs = crackProjectionPlane(crack, XYZ_wrs , A_WRS, camera_wrs);

        fprintf('Done. \n')

        if printFigs == 1
            %- Camera 1 projection
            null = zeros(length(crackPrj_wrs(1).width'),1);
            skel1PC_p = pointCloud(crackPrj_wrs(1).skel', 'Color',[100*crackPrj_wrs(1).width' null null]); %, , 'Intensity',crackPrj(1).width', 'Color', repmat([1 0 0], [length(crackPrj(1).skel'), 1])
            figure; hold on;
            grid on
            pcshow(skel1PC_p, 'VerticalAxis','Y', 'VerticalAxisDir','Down', 'MarkerSize',45)
            xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')
            plot(planeFitted_wrs, 'Color', [0.7 0.7 0.7])
            plot3(crackPrj_wrs(1).edgeR(1,:),crackPrj_wrs(1).edgeR(2,:),crackPrj_wrs(1).edgeR(3,:), '.g', 'MarkerSize', 10)
            plot3(crackPrj_wrs(1).edgeL(1,:),crackPrj_wrs(1).edgeL(2,:),crackPrj_wrs(1).edgeL(3,:), '.g', 'MarkerSize', 10)
            title('Crack projection from first image')

            %- Camera 2 projection
            null_2 = zeros(length(crackPrj_wrs(2).width'),1);
            skel2PC_p = pointCloud(crackPrj_wrs(2).skel', 'Color',[100*crackPrj_wrs(2).width' null null]); %, , 'Intensity',crackPrj(1).width', 'Color', repmat([1 0 0], [length(crackPrj(1).skel'), 1])
            figure; hold on;
            grid on
            pcshow(skel2PC_p, 'VerticalAxis','Y', 'VerticalAxisDir','Down', 'MarkerSize',45)
            xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')
            plot(planeFitted_wrs, 'Color', [0.7 0.7 0.7])
            plot3(crackPrj_wrs(2).edgeR(1,:),crackPrj_wrs(2).edgeR(2,:),crackPrj_wrs(2).edgeR(3,:), '.g', 'MarkerSize', 10)
            plot3(crackPrj_wrs(2).edgeL(1,:),crackPrj_wrs(2).edgeL(2,:),crackPrj_wrs(2).edgeL(3,:), '.g', 'MarkerSize', 10)
            title('Crack projection from second image')
            clearvars null null_2
        end
    end
%% Read Metashape Dense Cloud and Mesh

    if fitPlane == 0
    %- Read PLY mesh
        fprintf('Reading Metashape Mesh... ')
        [vertices, triang] = readMesh_ply(meshFlnm);
        vert1 = vertices(triang(:,1),:);
        vert2 = vertices(triang(:,2),:);
        vert3 = vertices(triang(:,3),:);

    % meshVertPC = pointCloud(vertices);
        if printFigs == 1
            figure('Name', 'Mesh from Metashape'); hold on
            title('Sparse Point Cloud Triangulation','fontsize',10)
            axis equal
            trisurf(triang,vertices(:,1),vertices(:,2),vertices(:,3),'facecolor','c','edgecolor','b') %plot della superficie
            axis vis3d
            view(3)
        end
        fprintf('Done.\n')

    %- Read Metashape Dense Cloud
        fprintf('Reading Metashape Dense Cloud... ')
        densePC = pointCloud(denseCloud);
        if printFigs == 1
            densePC.plot('Color', 'A.rgb','MarkerSize', 2); hold on
            xlabel('x'), ylabel('y'), zlabel('z')
            plotCamera('Location', X01_BBA, 'Orientation', R1_bba, 'Size', cameraSize, ...
                'Color', 'b', 'Label', '1', 'Opacity', 0);
            plotCamera('Location', X02_BBA, 'Orientation', R2_bba, 'Size', cameraSize, ...
                'Color', 'r', 'Label', '2', 'Opacity', 0);
        end
        fprintf('Done.\n')
%% Projective ray - Mesh Intersection

        fprintf('Projecting crack on the mesh... ')
    
        crackPrj = crackProjectionMesh(crack, vert1, vert2, vert3, cameras);
    
        fprintf('Done. \n')

    %- Plot
        if printFigs == 1
            %- Camera 1 projection
            figure; hold on;
            densePC.plot('Color', 'A.rgb','MarkerSize', 2); hold on
            xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')
%             pcshow(sparsePC_wrs, 'VerticalAxis','Z', 'VerticalAxisDir','Up', 'MarkerSize',45)
            plot3(crackPrj_wrs(1).skel(1,:),crackPrj_wrs(1).skel(2,:),crackPrj_wrs(1).skel(3,:), '.r', 'MarkerSize',5)
            plot3(crackPrj_wrs(1).edgeL(1,:),crackPrj_wrs(1).edgeL(2,:),crackPrj_wrs(1).edgeL(3,:), '.g', 'MarkerSize',5)
            plot3(crackPrj_wrs(1).edgeR(1,:),crackPrj_wrs(1).edgeR(2,:),crackPrj_wrs(1).edgeR(3,:), '.g', 'MarkerSize',5)

            %- Camera 2 projection
            figure; hold on;
            densePC.plot('Color', 'A.rgb','MarkerSize', 2); hold on
            xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')
%             pcshow(sparsePC_wrs, 'VerticalAxis','Z', 'VerticalAxisDir','Up', 'MarkerSize',45)
            plot3(crackPrj_wrs(2).skel(1,:),crackPrj_wrs(2).skel(2,:),crackPrj_wrs(2).skel(3,:), '.r', 'MarkerSize',5)
            plot3(crackPrj_wrs(2).edgeL(1,:),crackPrj_wrs(2).edgeL(2,:),crackPrj_wrs(2).edgeL(3,:), '.g', 'MarkerSize',5)
            plot3(crackPrj_wrs(2).edgeR(1,:),crackPrj_wrs(2).edgeR(2,:),crackPrj_wrs(2).edgeR(3,:), '.g', 'MarkerSize',5)
        end
    end
%% Export cloud
    pcwrite(pointCloud(crackPrj_wrs(1).skel', 'Intensity',crackPrj_wrs(1).width'), fullfile('pcOut','skel1_wrs.ply'));
    pcwrite(pointCloud(crackPrj_wrs(2).skel', 'Intensity',crackPrj_wrs(2).width'), fullfile('pcOut','skel2_wrs.ply'));
    pcwrite(pointCloud(crackPrj_wrs(1).edgeL'), fullfile('pcOut','left1_wrs.ply'));
    pcwrite(pointCloud(crackPrj_wrs(1).edgeR'), fullfile('pcOut','rigth1_wrs.ply'));
    pcwrite(pointCloud(crackPrj_wrs(2).edgeL'), fullfile('pcOut','left2_wrs.ply'));
    pcwrite(pointCloud(crackPrj_wrs(2).edgeR'), fullfile('pcOut','rigth2_wrs.ply'));

fprintf('End. \n\n')
end
