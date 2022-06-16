function [s] = detectCrack(I, varargin)
%DETECTCRACK
% 
%  SYNTAX:
%       [s] = detectCrack(I)
% 
%  INPUTS:
%   - I (mandatory) --> image
%   - msk (optional) --> masked binary image
%   - defFig (oprionale) --> Allow print figures
%   - warnings (par) --> Allows warning outputs [default=true]
% 
%  OUTPUT:
%   - s --> Structure with the image coordinates of skeleton, left and
%           right edges
% 
%  EXAMPLE: 
%       [s] = detectCrack(I);
% 
% Francesco Ioli, Alessandro Pinto 
% 09.05.2022 v1.0
%% Parse inputs
% - Set input default values
defMsk          = [];
defWarnFlag     = true; 
defVerbose      = false;
defFig          = true;

%- Create input parse
p = inputParser;
addRequired(p,'I',@isnumeric);
addOptional(p,'msk',defMsk,@isnumeric)
addParameter(p,'warnings',defWarnFlag,@islogical)
addParameter(p,'v',defVerbose,@islogical)
addParameter(p,'printfigure',defFig,@islogical)
parse(p, I, varargin{:})

%- Save parsed input vals to variables
I               = p.Results.I;
msk             = p.Results.msk;
warnFlag        = p.Results.warnings;
verbose         = p.Results.v;
printFigs       = p.Results.printfigure;
% mustBeNumericOrLogical(I)

%% deactivate warnings
if warnFlag == false
    warning off
end
%% Parameters

%- Image enhanchment
histAdj     = 1;
adaptHistEq = 0;

WienerFilt  = 1;
WienerWin   = [3 3];

medianFilt  = 0;
medianWin   = [5 5];

%- Binarization
binarSensitivity    = 0.2;

%- Skeletonization
shape = strel('disk', 5);

skelMinBranch       = 50;

%- Edge detection
edgeTresh           = [];
edgeDir             = 'both';

%- Parameters for skeleton-edge connection
q = 40;  % half research window for edge research
p = 3;   % half research window to evaluate local skeleton trend 
gauss_std = 2; % std for gaussian fitting
%% Load image 
Imsk = I;
Imsk = rgb2gray(Imsk);

%% Image enhanchment 

fprintf('Enhanching image: ... ')

%- Image enhanchment
if histAdj == 1
    fprintf('Equalizing histogram... ')
    Ien = imadjust(Imsk);
elseif adaptHistEq == 1
    Ien = adapthisteq(Imsk);
end
if printFigs == 1
    figure('Name','Before and after histogram equalization'); 
    imshowpair(Imsk, Ien, 'montage')
end

%- Wiener Filter
if WienerFilt == 1
    fprintf('Applying Wiener filter... ')
    Ien = wiener2(Ien, WienerWin);
    Ien = imadjust(Ien);
    if printFigs == 1
       figure('Name','Before and after Wiener filter'); 
       imshowpair(Imsk, Ien, 'montage')
    end
end

if medianFilt == 1
    fprintf('Applying median filter... ')    
    Ien =  medfilt2(Ien,medianWin);
    if printFigs == 1
        figure('Name','Before and after Median filter'); 
        imshowpair(I, Ien, 'montage')
    end
end

fprintf('Done.\n')
%% Binarization

%- binarizzazione
fprintf('Binarizing... ')    
bw = imbinarize(Imsk, 'adaptive', 'Sensitivity', binarSensitivity ,'ForegroundPolarity', 'dark');
bw = ~bw;
if printFigs == 1
    imshow(bw)
end
fprintf(' Done.\n')    

close all

%- mask to isolate binarized crack
if isempty(msk)
    figMsk = figure("Name",'Draw Mask'); 
    ax = gca;
    imshow(bw); hold on;
    roi = drawpolygon(ax);
    msk = createMask(roi, bw);
    close(figMsk)
    Imsk = bw == 1 & msk ==1;
    figure("Name",'Masked binary image'); imshow(Imsk)
end


%- Extract all connected components
ecc2flg = input('\nWhat connected components do you wanna extract? [1 = area; 2 = perimeter, 3 = extent; 0 = none]\n');
if ecc2flg == 1
   Imsk = bwpropfilt(Imsk,'Area',[150 1e5]);
   elseif ecc2flg == 2
     Imsk = bwpropfilt(Imsk, 'perimeter', [400 1e5]);
    elseif ecc2flg == 3
       Imsk = bwpropfilt(Imsk, 'extent', [0 0.4]);
end
if printFigs == 1
   figure("Name",'Results after filtering by connected components');imshow(Imsk)
end

ecc2flg = input('\nWhat connected components do you wanna extract? [1 = area; 2 = perimeter, 3 = extent; 0 = none]\n');
if ecc2flg == 1
   Imsk = bwpropfilt(Imsk,'Area',[150 1e5]);
   elseif ecc2flg == 2
     Imsk = bwpropfilt(Imsk, 'perimeter', [400 1e5]);
    elseif ecc2flg == 3
       Imsk = bwpropfilt(Imsk, 'extent', [0 0.4]);
end
if printFigs == 1
   figure("Name",'Results after filtering by connected components'); imshow(Imsk)
end

%- Second mask (if it is necessary)
msk2flg = input('\nDo you want to draw a second mask by hand? [1 = yes; 0 = no]\n');
if msk2flg == 1 
    figMsk2 = figure("Name",'Draw Mask'); 
    ax = gca;
    imshow(Imsk); hold on;
    roi = drawpolygon(ax);
    msk2 = createMask(roi, Imsk);
    close(figMsk2)
    Imsk = Imsk == 1 & msk2 ==1;
end

figure("Name",'Masked binary image'); imshow(Imsk)
%%  Morphological operations/Skeletonization

fprintf('Extracting skeleton: ... ')

%- Visual artificies removal
Imsk = bwmorph(Imsk,'spur');
Imsk = imfill(Imsk, 'holes');


%- Dilatation and erosion
Imsk = imclose(Imsk, shape);
Imsk = imfill(Imsk, 'holes');
if printFigs == 1
   imshow(Imsk)
end

close all

%- Extract Skeleton 
skel = bwskel(Imsk, 'MinBranchLength', skelMinBranch);
if printFigs == 1
    figure("Name",'Skeleton'); imshow(skel)
end

fprintf('Done.\n')

%% Edge detection

fprintf('Extracting borders: ... ')

edg2flg = input('\nWhich edge detection method do you wanna use? [1 = Canny; 2 = Sobel]\n');
if edg2flg == 1 
   bw_edge = edge(Imsk, "canny", edgeTresh);
   elseif edg2flg == 2
       bw_edge = edge(Imsk, "sobel", edgeTresh ,'direction', edgeDir);
end
if printFigs == 1
   figure("Name",'Edges'); imshow(bw_edge)
end

fprintf('Done.\n')
%% Joining skeleton and pixels

fprintf('Joining skeleton and pixels: ... ')

%- Variable initialization: skel = skeleton, edge_s = left edge pixel,
%edge_d = right edge pixel.
s = struct();
s.X_skel = 0;
s.Y_skel = 0;
s.X_edge_s = 0;
s.Y_edge_s = 0;
s.X_edge_d = 0;
s.Y_edge_d = 0;

k=1;   % cycle index

%- Masks used for edge pixel research
a = zeros(2*q+1,q);
b = ones(2*q+1,q+1);
z = zeros(q,2*q+1);
d = ones(q+1,2*q+1);
v = ones([2*q+1 1]);
h = ones([1 2*q+1]);

sins = [b,a]; % Isolate edge on the left of the central pixel.
des = [a,b]; % Isolate edge on the right of the central pixel.
su = [d;z]; % Isolate edge above the central pixel.
giu = [z;d]; % Isolate edge under the central pixel.

mask_v = zeros([2*q+1 2*q]);
mask_h = zeros([2*q 2*q+1]);
mask_v = [mask_v(:,1:q),v,mask_v(:,q+1:end)]; % Vertical research mask.
mask_h = [mask_h(1:q,:);h;mask_h(q+1:end, :)]; % Horizontal research mask.

%- Add black borders to images
bord = zeros(size (skel)+(2*q), 'like', skel);
bord(q+1:size(skel,1)+q,q+1:size(skel,2)+q)=skel;
bord_edg = zeros(size (bw_edge)+2*q, 'like', bw_edge);
bord_edg(q+1:size(bw_edge,1)+q,q+1:size(bw_edge,2)+q)=bw_edge;


for i = q+1:size(bord,1)-q
   for j = q+1:size(bord,2)-q
       if bord(i,j) == 1
%           lastwarn('')
          IA = uint8(bord(i-p:i+p,j-p:j+p)); % Open a research window around the [i,j] pixel in the skeleton image.
          [y,x] = find(IA);
          c = polyfit(x,y,1); % Search of the skeleton local linear trend.
          angle = atand(c(1));
%           msg = lastwarn;
%           if ~isempty(msg)
%                 continue
%           end
          if angle >= -10 && angle <= 10 % Horizontal skeleton, vertical research no rotation needed
             SA = double(bord_edg(i-q:i+q,j-q:j+q)); % Open a research window around the [i,j] pixel in the edge image.
             SA_s = su.*SA; % Search of the left edge pixel.
             temp_s= SA_s + mask_v;
             temp_s = temp_s == 2;
             ind_s = find(temp_s,1);
             if length(ind_s) < 1 % Skip if no border found.
                continue
             end
             [y_s,x_s] = ind2sub(size(SA_s),ind_s); % Get edge pixel coordinates.
             u_s = [x_s-41 y_s-41]; % Left edge vector.
             SA_g = giu.*SA; % Search of the right edge pixel.
             temp_g = SA_g + mask_v;
             temp_g = temp_g == 2;
             ind_g = find(temp_g,1);
             if length(ind_g) < 1 % Skip if no border found.
                continue
             end
             [y_d,x_d] = ind2sub(size(SA_g),ind_g); % Get edge pixel coordinates.
             u_d = [x_d-41 y_d-41]; % Right edge vector.
             s(k).Y_skel = i-40; % Repositioning the coordinates in the correct image system.
             s(k).X_skel = j-40;
             s(k).X_edge_s = j - 40;
             s(k).Y_edge_s= i - 40 - norm(u_s);
             s(k).X_edge_d =j - 40;
             s(k).Y_edge_d= i - 40 + norm(u_d);
             k = k+1;
          elseif angle <= -80 || angle >= 80 % Vertical skeleton, horizontal research no rotation.
             SA = double(bord_edg(i-q:i+q,j-q:j+q)); % Open a research window around the [i,j] pixel in the edge image.
             SA_s = sins.*SA; % Search of the left edge pixel.
             temp_s = SA_s + mask_h;
             temp_s = temp_s == 2;
             ind_s = find(temp_s,1);
             if length(ind_s) < 1 % Skip if no border found.
                continue
             end
             [y_s,x_s] = ind2sub(size(SA_s),ind_s);
             u_s = [x_s-41 y_s-41]; % Left edge vector.
             SA_d = des.*SA; % Search of the right edge pixel.
             temp_d = SA_d + mask_h;
             temp_d = temp_d == 2;
             ind_d = find(temp_d,1);
             if length(ind_d) < 1 % Skip if no border found.
                continue
             end
             [y_d,x_d] = ind2sub(size(SA_d),ind_d);
             u_d = [x_d-41 y_d-41]; % Right edge vector.
             s(k).Y_skel = i-40; % Repositioning the coordinates in the correct image system.
             s(k).X_skel = j-40;
             s(k).X_edge_s = j - 40 - norm(u_s);
             s(k).Y_edge_s= i - 40;
             s(k).X_edge_d =j - 40 + norm(u_d);
             s(k).Y_edge_d= i - 40;
             k = k+1;
          else % All others angle values need a rotation to obtain the correct edges.
             angler = -sign(angle)*(90 - abs(angle)); % Perpendicular direction of research.
             SA = double(bord_edg(i-q:i+q,j-q:j+q)); % Open a research window around the [i,j] pixel in the edge image.
             rot = imrotate(SA,angler,'bilinear','crop'); % Rotation -> interpolation method: bilinear.
             rot_s = sins.*rot; % Search of the left edge pixel.
             rot_s_r = imresize(rot_s, 2, 'bilinear'); % Resizing to have correct distances -> interpolation method: bilinear.
             if find( rot_s_r((2*q)+1,:)) == 0 % Skip if no border found.
                    continue
             end
             f_s = fit([1:size(rot_s_r,1)]' , rot_s_r((2*q)+1,:)' ,'gauss1', 'upper', [+Inf +Inf gauss_std]); % Gaussian fitting along the central row to find the most probable position of the edge pixel.
             x_s = f_s.b1/2;
             y_s = q+1;
             u_s = [x_s-41 y_s-41]; % Left edge vector.
             rot_d = des.*rot; % Search of the right edge pixel.
             rot_d_r = imresize(rot_d, 2, 'bilinear'); % Resizing to have correct distances -> interpolation method: bilinear.
             if find( rot_d_r((2*q)+1,:)) == 0 % Skip if no border found.
                    continue
             end
             f_d = fit([1:size(rot_d_r,1)]' , rot_d_r((2*q)+1,:)' ,'gauss1', 'upper', [+Inf +Inf gauss_std]); % Gaussian fitting along the central row to find the most probable position of the edge pixel.
             x_d = f_d.b1/2;
             y_d  = q+1;
             u_d = [x_d-41 y_d-41]; % Right edge vector.
             s(k).Y_skel = i-40; % Repositioning the coordinates in the correct image system.
             s(k).X_skel = j-40;
             s(k).X_edge_s = j - 40 + (cosd(angler)*u_s(1) - sind(angler)*u_s(2)); % Apply a rotation to get the correct pixel position.
             s(k).Y_edge_s= i - 40 + (sind(angler)*u_s(1) + cosd(angler)*u_s(2));
             s(k).X_edge_d =j - 40 + (cosd(angler)*u_d(1) - sind(angler)*u_d(2));
             s(k).Y_edge_d= i - 40 + (sind(angler)*u_d(1) + cosd(angler)*u_d(2));
             k = k+1;
          end
       end
   end
end

fprintf('Done.\n')

close all

%% Plot recognized crack on the image
if printFigs == 1

 fprintf('Printing your recognized crack: ... ')

 punti_s = [s.X_edge_s; s.Y_edge_s]';
 punti_d = [s.X_edge_d; s.Y_edge_d]';
 punti_sk = [s.X_skel; s.Y_skel]';

 figure("Name",'Recognized skeleton and edges'); imshow(I)
 hold on;
 scatter(punti_sk(:,1), punti_sk(:,2), 3, 'r')
 hold on;
 scatter(punti_s(:,1), punti_s(:,2), 3, 'g')
 hold on;
 scatter(punti_d(:,1), punti_d(:,2), 3, 'g')
 legend('Skeleton', 'Edges')

 fprintf('Done.\n')
end

end