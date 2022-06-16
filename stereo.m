%% Lettura immagini e rimozione delle distorsioni

im1 = imread('DJI_0832.tif');
im2 = imread('DJI_0835.tif');

load('target_mano_cal.mat')  % Lettura parametri camera, se già calibrata

%Rimozioni distorsioni
im1u = undistortImage(im1, cameraParams);
im2u = undistortImage(im2, cameraParams);
imshowpair(im1u, im2u, 'montage')

% Conversione scala di grigi, per ricerca punti SIFT
I1gray = rgb2gray(im1u);
I2gray = rgb2gray(im2u);

%% Ricerca punti di interesse

% Utilizzo dell'operatore SIFT per ricercare i punti di interesse nelle
% immagini. Ricerca più stringente per valutare in maniera più robusta la
% matrice essenziale e l'orientamento esterno.

% Miglioramento del contrasto con equalizzazione adattiva degli istogrammi
I1 = adapthisteq(I1gray); % Miglioramento del contrasto con equalizzaizone adattiva dell'istogramma
I2 = adapthisteq(I2gray);

% Ricerca delle caratteristice SIFT nelle immagini. I valori di "Threshold" e "Sigma" possono essere variati in basi a quanti punti di interesse si vogliono ricercare.
blobs1 = detectSIFTFeatures(I1, 'ContrastThreshold', 0.06, 'EdgeThreshold', 3, 'Sigma', 2); % "ContrastThreshold" [0,1] più alto meno punti trova. "EdgeThreshold" più alto più punti trova.
blobs2 = detectSIFTFeatures(I2, 'ContrastThreshold', 0.06, 'EdgeThreshold', 3, 'Sigma', 2); % "Sigma" della gaussiana in input valori tipici [1,2], ridurre sigma se immagine sfocata.

% figure;
% imshow(im1);
% hold on;
% plot(selectStrongest(blobs1, 100));
% title('Thirty strongest SURF features in I1');


% Match dei punti omologhi tra le immagini
[features1, validBlobs1] = extractFeatures(I1, blobs1);
[features2, validBlobs2] = extractFeatures(I2, blobs2);
indexPairs = matchFeatures(features1, features2, 'MatchThreshold', 2, 'Unique', true); % "MatchThreshold" rappresenta la percentuale di distanza da un match perfetto, "Metric" scegliere tra SSD e SAD, "Unique" se true trova un unico match tra due features
matchedPoints1 = validBlobs1(indexPairs(:,1),:);
matchedPoints2 = validBlobs2(indexPairs(:,2),:);
% figure;
% showMatchedFeatures(im1, im2, matchedPoints1, matchedPoints2, 'montage');
% legend('Presunti punti Omologhi in I1', 'Presunti punti omologhi in I2');
%% Matrice fondamentale

% Da utilizzare nel caso non è stata effettuata precedentemente la
% calibrazione della camera.

% Rimozione outliers
% [fMatrix, epipolarInliers, status] = estimateFundamentalMatrix(...
%   matchedPoints1, matchedPoints2, 'Method', 'RANSAC', ...
%   'NumTrials', 10000, 'DistanceThreshold', 0.5, 'Confidence', 99);
% 
% if status ~= 0 || isEpipoleInImage(fMatrix, size(im1)) ...
%   || isEpipoleInImage(fMatrix', size(im2))
%   error(['Either not enough matching points were found or '...
%          'the epipoles are inside the images. You may need to '...
%          'inspect and improve the quality of detected features ',...
%          'and/or improve the quality of your images.']);
% end
% 
% inlierPoints1 = matchedPoints1(epipolarInliers, :);
% inlierPoints2 = matchedPoints2(epipolarInliers, :);
% 
% figure;
% showMatchedFeatures(im1, im2, inlierPoints1, inlierPoints2, 'montage');
% legend('Inlier points in I1', 'Inlier points in I2');
% 
% % Rettifica immagine e orientamento esterno delle camere
% [t1, t2] = estimateUncalibratedRectification(fMatrix, ...
%   inlierPoints1, inlierPoints2, size(im2));
% tform1 = projective2d(t1);
% tform2 = projective2d(t2);
% 
% 
% [I1Rect, I2Rect] = rectifyStereoImages(I1, I2, tform1, tform2, 'OutputView','full');
% imshowpair(I1Rect, I2Rect, 'montage')
% figure;
% imshow(stereoAnaglyph(I1Rect, I2Rect));
% title('Rectified Stereo Images (Red - Left Image, Cyan - Right Image)');
%% Matrice essenziale e ricerca degli inlier

% Calcolo della matrice essenziale e rimozione degli outlier dai punti
% omologhi.

[EMatrix, epipolarInliers_e] = estimateEssentialMatrix(...
  matchedPoints1, matchedPoints2, cameraParams, ...
  'MaxNumTrials', 10000, 'MaxDistance', 0.5, 'Confidence', 99.99); % Utilizza MSAC per rimuovere gli outlier

inlierPoints1 = matchedPoints1(epipolarInliers_e, :);
inlierPoints2 = matchedPoints2(epipolarInliers_e, :);
% 
% figure;
% showMatchedFeatures(im1, im2, inlierPoints1, inlierPoints2, 'blend');
%% Ricostruzione Posizioni camere e rettifica delle immagini

% Grazie alla matrice essenziale si ricava l'orientamento esterno delle
% camere. Sistema di riferimento locale, con origine nel centro di presa
% della prima camera.

[relativeOrientation, relativeLocation] = relativeCameraPose(EMatrix, cameraParams, inlierPoints1,inlierPoints2); % Posizione relativa della camera 2 rispetto alla 1
[rotationMatrix,translationVector] = cameraPoseToExtrinsics(relativeOrientation,relativeLocation); % Orientamento esterno camera 2
tform2 = rigid3d(rotationMatrix, translationVector); % Orientamento esterno camera 2, salvato in maniera diversa
% stereoParams = stereoParameters(cameraParams,cameraParams,tform2); % Servono solo per rettifica
% 
% [I1Rect_1, I2Rect_2] = rectifyStereoImages(im1, im2, stereoParams); % Se rettificate correttamente, pixel uguali alla stessa altezza tra le due immagini
% imshowpair(I1Rect_1, I2Rect_2, 'montage')
%% Triangolazione punti omologhi

% Ricerca di maggiori punti SIFT per creare nuvola sparsa.
blobs3 = detectSIFTFeatures(I1, 'ContrastThreshold', 0.04, 'EdgeThreshold', 3, 'Sigma', 2); % "ContrastThreshold" [0,1] più alto meno punti trova. "EdgeThreshold" più alto più punti trova.
blobs4 = detectSIFTFeatures(I2, 'ContrastThreshold', 0.04, 'EdgeThreshold', 3, 'Sigma', 2);
% Match dei punti
[features1, validBlobs1] = extractFeatures(I1, blobs3);
[features2, validBlobs2] = extractFeatures(I2, blobs4);
indexPairs = matchFeatures(features1, features2, 'MatchThreshold', 2, 'Unique', true); % "MatchThreshold" rappresenta la percentuale di distanza da un match perfetto. "Metric" scegliere tra SSD e SAD. "Unique" se true trova un unico match tra due features
matchedPoints1 = validBlobs1(indexPairs(:,1),:);
matchedPoints2 = validBlobs2(indexPairs(:,2),:);

% Triangolazione punti omologhi
tform = rigid3d; % orientamento camera 1, che è nell'origine del sistema di riferimento locale e matrice di rotazione pari all'identità
camMatrix = cameraMatrix(cameraParams, tform); % matrice camera 1
camMatrix2 = cameraMatrix(cameraParams, rotationMatrix, translationVector); % matrice camera 2
xyzPoints= triangulate(matchedPoints1, matchedPoints2, camMatrix, camMatrix2); % punti triangolati

% Triangolazione punti doppi per scalare il modello. Riconoscimento marker
% nelle immagini e successiva triangolazione di essi.
% doppi_28 = zeros(2,2);
% doppi_29 = zeros(2,2);
% [doppi_28,doppi_29] = cpselect(im1u,im2u, doppi_28, doppi_29,'Wait',true)
% xyzPoints_dop= triangulate(doppi_28, doppi_29, camMatrix, camMatrix2); % coordinate marker triangolati
% ms60 = load('marker.txt'); % Coordinate misurate dei marker

% sz = 20;
% figure;
% imshow(im1u)
% hold on;
% scatter(doppi_28(:,1), doppi_28(:,2), sz,"red", 'filled')
% title('Marker utilizzati per scalare il modello')

% Nuvola sparsa di punti non scalata in sistema locale
ptCloud = pointCloud(xyzPoints); % Crea nuvola punti
ptCloud_dn = pcdenoise(ptCloud, 'Threshold', 0, 'NumNeighbors', 20); % rimuove rumore, non del tutto funzionante la funzione.
cameraSize = 0.3;
figure
plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
hold on
grid on
plotCamera('Location', relativeLocation, 'Orientation', relativeOrientation, 'Size', cameraSize, ...
    'Color', 'b', 'Label', '2', 'Opacity', 0);
pcshow(ptCloud_dn, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
% hold on;
% pcshow(xyzPoints_dop, 'r', 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
%     'MarkerSize', 100)


% Scalare il modello conoscendo la distanza tra 2 marker
dist_imm = sqrt((xyzPoints_dop(1,1) - xyzPoints_dop(2,1))^2 + (xyzPoints_dop(1,2) - xyzPoints_dop(2,2))^2 + (xyzPoints_dop(1,3) - xyzPoints_dop(2,3))^2); % distanza tra marker triangolati
dist_real = sqrt((ms60(1,1) - ms60(2,1))^2 + (ms60(1,2) - ms60(2,2))^2 + (ms60(1,3) - ms60(2,3))^2); % distanza reale tra le coordinate misurate
scalefactor = dist_real/dist_imm; % metri
scaledpoints= scalefactor*xyzPoints; % moltiplicare la variabile con le coordinate dei punti, poi ricreare point cloud
ptCloud_sc = pointCloud(scaledpoints);
relativeLocation_Sc = scalefactor* relativeLocation; % posiziona la camera 2 nella giusta posizione
ptCloud_sc_dn = pcdenoise(ptCloud_sc, 'Threshold', 0, 'NumNeighbors', 20); % Rimozione rumore da nuvola di punti

dist_imm = sqrt((0 - -0.998980767297135)^2 + (0 - -0.0432759441820958)^2 + (0 - 0.0128304023933939)^2); % distanza tra marker triangolati
dist_real = sqrt((997.150467 - 998.565244)^2 + (1017.608218 - 1017.264502)^2 + (104.634822 - 104.593230)^2); % distanza reale tra le coordinate misurate
scalefactor = dist_real/dist_imm; % metri

% Nuvola sparsa scalata
cameraSize = 0.3;
figure
plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
hold on
grid on
plotCamera('Location', relativeLocation_Sc, 'Orientation', relativeOrientation, 'Size', cameraSize, ...
    'Color', 'b', 'Label', '2', 'Opacity', 0);
pcshow(ptCloud_sc_dn, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
xlabel('X');
ylabel('Z');
zlabel('Y');

%% Richiama punti delle fessure riconosciuti in precedenza

% I punti riconosciuti vengono divisi in punti dello scheltro, punti dei
% bordi a sinistra e punti dei bordi a destra dello scheletro.

load('DJI_0828u3.mat')
punti_0828_s = [s0828.X_edge_s; s0828.Y_edge_s]';
punti_0828_d = [s0828.X_edge_d; s0828.Y_edge_d]';
punti_0828_sk = [s0828.X_skel; s0828.Y_skel]';

load('DJI_0829u3.mat')
punti_0829_s = [s0829.X_edge_s; s0829.Y_edge_s]';
punti_0829_d = [s0829.X_edge_d; s0829.Y_edge_d]';
punti_0829_sk = [s0829.X_skel; s0829.Y_skel]';

punti_s = [s.X_edge_s; s.Y_edge_s]';
punti_d = [s.X_edge_d; s.Y_edge_d]';
punti_sk = [s.X_skel; s.Y_skel]';

%% Fitting del piano medio interpolante sulla nuvola sparsa

% Interpolato piano medio sulla nuvola sparsa per proiettarci sopra la
% fessura i punti di scheletro e bordi.

[model3, inlieridx3] = pcfitplane(ptCloud_sc_dn, 0.001); % metri, "0.005" distanza massima tra il piano e i punti della nuvola. Usa algoritmo MSAC per trovare il piano.
plane = select(ptCloud_sc_dn,inlieridx3); % seleziona solo i punti della nuvola entro la distanza dal piano
col = [100 100 100];
plane.Color = uint8(repmat(col, plane.Count,1));

% Fit di piani locali interpolanti, metodo alternativo per trovare la
% superficie sul quale proiettare i punti.

% lambda  = 1;
% planeFitMaxDist = 0.0005;
% kNearPts        = 20;
% % Aloc            = zeros(4, length(targets(1).m));
% for tar = 1:length(point_3d_28)
%     fprintf('Fitting plane around target %i...', tar)
%     [indices,dists] = findNearestNeighbors(ptCloud_sc_dn, point_3d_28(:,tar)', kNearPts);
%     locPC = select(ptCloud_sc_dn,indices);
%     [planeFitted_WRS, planeFitInIdx_WRS, planeFitOutIdx_WRS, planeFitErr_WRS] = pcfitplane(locPC, planeFitMaxDist);
%     fprintf('Inlier points: %i (%.0f%%); Outlier points: %i; Mean fit error: %.4f\n', ...
%         length(planeFitInIdx_WRS), length(planeFitInIdx_WRS)/length(locPC.Location)*100, length(planeFitOutIdx_WRS), planeFitErr_WRS)  
%     Aloc       = [planeFitted_WRS.Parameters(1:3) -planeFitted_WRS.Parameters(4)]';
%    
%     point_3d_28_new(:,tar)    = rayPlaneIntersection(ensure_homogeneous(hom_sk_28(:,tar)), cameras(1), Aloc, lambda);
% %     targetsProj_wrs(2).XYZ(:,tar)    = rayPlaneIntersection(ensure_homogeneous(targets(2).m(:,tar)), cameras(2), Aloc, lambda);
% end

%% Coordinate omogenee: Fessura 0828
un = ones(length(punti_s(:,1)),1);
hom_s = [punti_s(:,1) punti_s(:,2) un]';
hom_d = [punti_d(:,1) punti_d(:,2) un]';
hom_sk = [punti_sk(:,1) punti_sk(:,2) un]';

%% Coordinate omogenee: Fessura 0829
un = ones(length(punti_0829_s(:,1)),1);
hom_s_29 = [punti_0829_s(:,1) punti_0829_s(:,2) un]';
hom_d_29 = [punti_0829_d(:,1) punti_0829_d(:,2) un]';
hom_sk_29 = [punti_0829_sk(:,1) punti_0829_sk(:,2) un]';

%% Proiezione punti con metodo Plucker: intersezione raggio-piano

% Scrivendo i punti in coordinate omogenee, si trova la linea di Plucker
% passante tra il punto riconosciuto nell'immagine e il punto all'infinito
% associato. L'intersezione della linea col piano permette di trovare la
% coordinate 3D del punto.

A = [model3.Parameters(1:3) -model3.Parameters(4)]'; % Piano in coordinate omogenee
B = eye(3); % rotazione camera 1
KR_28 = inv(cameraParams.IntrinsicMatrix' * B); % K*R camera 1, passaggio intermedio
s = [0 -A(3) A(2); A(3) 0 -A(1); -A(2) A(1) 0]; % matrice skew del piano, per intersezione
wedge = [A(4)* diag([1 1 1]), -s; -A(1:3)', [0 0 0]]; % operatore wedge del piano, per intersezione
point_3d_28 = zeros(size(hom_sk)); % scheletro
for i = 1:length(hom_sk)
%     i = 1;
    point_inf = [[0 0 0]' + KR_28*hom_sk(:,i);0]; % punto all'infinito
    point = [[0 0 0]' + KR_28*hom_sk(:,i);1];  % punto lungo raggio proiettivo passante per il punto che vogliamo proiettare
    linea_pluc = [point_inf(4)*[point(1:3)] - point(4)*point_inf(1:3); cross(point_inf(1:3),point(1:3))]; % Linea di Plucker tra i 2 punti
    point_hom = wedge*linea_pluc; % intersezione linea-piano
    point_3d_28(:,i) = -[point_hom(1:3)/point_hom(4)]; % coordinate 3D del punto
end


point_3d_28_edg_s = zeros(size(hom_s)); % bordo sinistro
for i = 1:length(hom_s)
    point_inf = [[0 0 0]' + KR_28*hom_s(:,i);0]; 
    point = [[0 0 0]' + KR_28*hom_s(:,i);1]; 
    linea_pluc = [point_inf(4)*[point(1:3)] - point(4)*point_inf(1:3); cross(point_inf(1:3),point(1:3))];
    point_hom = wedge*linea_pluc;
    point_3d_28_edg_s(:,i) = -[point_hom(1:3)/point_hom(4)];
end


point_3d_28_edg_d = zeros(size(hom_d)); % bordo destro
for i = 1:length(hom_d)
    point_inf = [[0 0 0]' + KR_28*hom_d(:,i);0]; 
    point = [[0 0 0]' + KR_28*hom_d(:,i);1]; 
    linea_pluc = [point_inf(4)*[point(1:3)] - point(4)*point_inf(1:3); cross(point_inf(1:3),point(1:3))];
    point_hom = wedge*linea_pluc;
    point_3d_28_edg_d(:,i) = -[point_hom(1:3)/point_hom(4)];
end

KR_29 = inv(cameraParams.IntrinsicMatrix' * relativeOrientation); % KR camera 2, passaggio intermedio
point_3d_29 = zeros(size(hom_sk_29)); % scheletro
for i = 1:length(hom_sk_29)
    point_inf = [KR_29*hom_sk_29(:,i);0]; % punto all'infinito
    point = [-relativeLocation_Sc' + KR_29*hom_sk_29(:,i);1]; % punto lungo raggio proiettivo passante per il punto che vogliamo proiettare
    linea_pluc = [point_inf(4)*[point(1:3)] - point(4)*point_inf(1:3); cross(point_inf(1:3),point(1:3))]; % Linea di Plucker tra i 2 punti
    point_hom = wedge*linea_pluc; % intersezione linea-piano
    point_3d_29(:,i) = -[point_hom(1:3)/point_hom(4)]; % coordinate 3D del punto
end


point_3d_29_edg_s = zeros(size(hom_s_29)); % bordo sinistro
for i = 1:length(hom_s_29)
    point_inf = [KR_29*hom_s_29(:,i);0];
    point = [-relativeLocation_Sc' + KR_29*hom_s_29(:,i);1]; 
    linea_pluc = [point_inf(4)*[point(1:3)] - point(4)*point_inf(1:3); cross(point_inf(1:3),point(1:3))];
    point_hom = wedge*linea_pluc;
    point_3d_29_edg_s(:,i) = -[point_hom(1:3)/point_hom(4)];
end

point_3d_29_edg_d = zeros(size(hom_d_29)); % bordo destro
for i = 1:length(hom_d_29)
    point_inf = [KR_29*hom_d_29(:,i);0];
    point = [-relativeLocation_Sc' + KR_29*hom_d_29(:,i);1]; 
    linea_pluc = [point_inf(4)*[point(1:3)] - point(4)*point_inf(1:3); cross(point_inf(1:3),point(1:3))];
    point_hom = wedge*linea_pluc;
    point_3d_29_edg_d(:,i) = -[point_hom(1:3)/point_hom(4)];
end

%% Proiezione fessura sul piano

% Nuvola sparsa della scena con sovrapposti piano interpolante, e le
% proiezioni delle fessure dalle due immagini

cameraSize = 0.3;
figure
plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
hold on
grid on
plotCamera('Location', relativeLocation_Sc, 'Orientation', relativeOrientation, 'Size', cameraSize, ...
    'Color', 'b', 'Label', '2', 'Opacity', 0);
pcshow(plane,'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
title('Proiezione della fessura da una immagine')
xlabel('X (m)');
ylabel('Z (m)');
zlabel('Y (m)');
% xlim([-2.5 1])
% ylim([-2 2])
% zlim([-1 5])
% Piano interpolante
hold on;
plot(model3, 'Color', [0.7 0.7 0.7])
% Fessura immagine 1
hold on;
plot3(point_3d_28(1,:), point_3d_28(2,:), point_3d_28(3,:), 'o', 'Color', 'r', 'MarkerSize', 1)
hold on;
plot3(point_3d_28_edg_s(1,:), point_3d_28_edg_s(2,:), point_3d_28_edg_s(3,:), 'o', 'Color', [0 0.7 0], 'MarkerSize', 1)
hold on;
plot3(point_3d_28_edg_d(1,:), point_3d_28_edg_d(2,:), point_3d_28_edg_d(3,:), 'o', 'Color', [0 0.7 0], 'MarkerSize', 1)
hold on;
% Fessura immagine 2
% plot3(point_3d_29(1,:), point_3d_29(2,:), point_3d_29(3,:), 'o', 'Color', 'r', 'MarkerSize', 1)
% hold on;
% plot3(point_3d_29_edg_s(1,:), point_3d_29_edg_s(2,:), point_3d_29_edg_s(3,:), 'o', 'Color', 'g', 'MarkerSize', 1)
% hold on;
% plot3(point_3d_29_edg_d(1,:), point_3d_29_edg_d(2,:), point_3d_29_edg_d(3,:), 'o', 'Color', 'g', 'MarkerSize', 1)

%% Salvataggio modelli 3D delle fessure

% Per ogni punti dello scheletro si stima la larghezza valutata come
% distanza euclidea tra i punti dei bordi associati.

larg28= zeros(length(point_3d_28),1);
for i = 1:length(point_3d_28)
    larg28(i) = sqrt((point_3d_28_edg_s(1,i) - point_3d_28_edg_d(1,i))^2 + (point_3d_28_edg_s(2,i) - point_3d_28_edg_d(2,i))^2 + (point_3d_28_edg_s(3,i) - point_3d_28_edg_d(3,i))^2);
end

larg29= zeros(length(point_3d_29),1);
for i = 1:length(point_3d_29)
    larg29(i) = sqrt((point_3d_29_edg_s(1,i) - point_3d_29_edg_d(1,i))^2 + (point_3d_29_edg_s(2,i) - point_3d_29_edg_d(2,i))^2 + (point_3d_29_edg_s(3,i) - point_3d_29_edg_d(3,i))^2);
end

% Trasforma i vari scheletri e bordi in nuvole di punti e li salva come
% file ".ply"
skel_28 = pointCloud(point_3d_28', 'Intensity',larg28); % Salva la larghezza come valore associato ad ogni punto dello scheletro
s_28 = pointCloud(point_3d_28_edg_s');
d_28 = pointCloud(point_3d_28_edg_d');
skel_29 = pointCloud(point_3d_29','Intensity',larg29); % Salva la larghezza come valore associato ad ogni punto dello scheletro
s_29 = pointCloud(point_3d_29_edg_s');
d_29 = pointCloud(point_3d_29_edg_d');
pcwrite(skel_28, 'Skel_283.ply');
pcwrite(s_28, 'S_283.ply');
pcwrite(d_28, 'D_283.ply');
pcwrite(skel_29, 'Skel_293.ply');
pcwrite(s_29, 'S_293.ply');
pcwrite(d_29, 'D_293.ply');
%% Analisi marker

% Analisi sui marker effettuata per valutare l'accuratezza della proiezione

% Triangolazione dei marker visibili nella scena 
doppi_28_m = zeros(4,2);
doppi_29_m = zeros(4,2);
[doppi_28_m,doppi_29_m] = cpselect(im1u,im2u,'Wait',true)
xyzPoints_dop_m = triangulate(doppi_28_m, doppi_29_m, camMatrix, camMatrix2); % Triangolazione dei marker
ms60_r = load('marker_rimanenti.txt'); % coordinate misurate dei marker
xyzPoints_dop_m_sc = scalefactor*xyzPoints_dop_m; % marker scalati

% coordinate omogenee dei marker
un_m = ones(5,1);
hom_m_28 = [doppi_28_m(:,1) doppi_28_m(:,2) un_m]';
hom_m_29 = [doppi_29_m(:,1) doppi_29_m(:,2) un_m]';

% Proiezione marker su piano dalle due immagini con metodo Plucker
point_3d_28_m = zeros(size(hom_m_28)); % Marker riconosciuti in immagine 1
for i = 1:length(hom_m_28)
    point_inf = [[0 0 0]' + KR_28*hom_m_28(:,i);0];
    point = [[0 0 0]' + KR_28*hom_m_28(:,i);1]; 
    linea_pluc = [point_inf(4)*[point(1:3)] - point(4)*point_inf(1:3); cross(point_inf(1:3),point(1:3))];
    point_hom = wedge*linea_pluc;
    point_3d_28_m(:,i) = -[point_hom(1:3)/point_hom(4)]; % coordinate 3D dei marker
end

point_3d_29_m = zeros(size(hom_m_29)); % Marker riconosciuti in immagine 2
for i = 1:length(hom_m_29)
    point_inf = [KR_29*hom_m_29(:,i);0];
    point = [-relativeLocation_Sc' + KR_29*hom_m_29(:,i);1]; 
    linea_pluc = [point_inf(4)*[point(1:3)] - point(4)*point_inf(1:3); cross(point_inf(1:3),point(1:3))];
    point_hom = wedge*linea_pluc;
    point_3d_29_m(:,i) = -[point_hom(1:3)/point_hom(4)]; % coordinate 3D dei marker
end

% Nuvola dei marker triangolati + proiezioni dei marker dalle singole
% immagini.
ptCloud_m = pointCloud(xyzPoints_dop_m_sc);
cameraSize = 0.3;
figure
plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
hold on
grid on
plotCamera('Location', relativeLocation_Sc, 'Orientation', relativeOrientation, 'Size', cameraSize, ...
    'Color', 'b', 'Label', '2', 'Opacity', 0);
% pcshow(ptCloud_sc_dn ,'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
%     'MarkerSize', 45);
pcshow(ptCloud_m ,'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 100); % Marker triangolati
title('Proiezione marker')
xlabel('X');
ylabel('Z');
zlabel('Y');
hold on;
plot3(point_3d_28_m(1,:), point_3d_28_m(2,:), point_3d_28_m(3,:), '+', 'Color', 'r', 'MarkerSize', 25) % Marker proiettati camera 1
hold on;
plot3(point_3d_29_m(1,:), point_3d_29_m(2,:), point_3d_29_m(3,:), '+', 'Color', 'g', 'MarkerSize', 25) % Marker proiettati camera 2

% save('Marker_analisi.mat')
%% Salvataggio modelli 3D marker

marker_28 = pointCloud(point_3d_28_m');
marker_29 = pointCloud(point_3d_29_m');
pcwrite(marker_28, 'marker_28.ply');
pcwrite(marker_29, 'marker_29.ply');
%% Analisi marker residui

% Differenza tra le coordinate dei marker proiettati, dopo
% georeferenziazione e coordinate misurate con multi-station

marker_28_w = load('marker_28_word.txt'); % Caricare coordinate post georeferenziazione
marker_29_w = load('marker_29_word.txt');
marker_28_w_o = marker_28_w; % Rimossa una colonna non di coordinate
marker_29_w_o = marker_29_w; % Rimossa una colonna non di coordinate

diff_28_r_o = marker_28_w_o - ms60_r; % Differenze tra proiezione da camera 1 e coordinate misurate
diff_29_r_o = marker_29_w - ms60_r; % Differenze tra proiezione da camera 2 e coordinate misurate


a_28_o = struct(); % struttura con statitische relative alla differenza tra camera 1 e coordinate misurate
a_28_o.max_o = max(diff_28_r_o);
a_28_o.min= min(diff_28_r_o);
a_28_o.mean = mean(diff_28_r_o); % togliere 0.0015 a media y -> spessore medio target
a_28_o.std= std(diff_28_r_o);
a_28_o.rms = rms(diff_28_r_o);

a_29_o = struct(); % struttura con statitische relative alla differenza tra camera 2 e coordinate misurate
a_29_o.max = max(diff_29_r_o);
a_29_o.min= min(diff_29_r_o);
a_29_o.mean = mean(diff_29_r_o); % togliere 0.0015 a media y -> spessore  medio target
a_29_o.std= std(diff_29_r_o);
a_29_o.rms = rms(diff_29_r_o);