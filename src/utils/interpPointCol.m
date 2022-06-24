function col = interpPointCol(pointsXYZ, im, P, cameraPars, do_viz)
% interpPointCol build rotation matrix from
% 
% col = interpPointCol(pointsXYZ, im, cam)
%  Inputs:  
%   - 3xN matrix with 3d world points coordinates
%   - image
%   - camera projection matrix P (3x4 matrix)
% 
% Output: 3xN colour matrix 
%
% Francesco Ioli
% 2022.04.20

if ~exist('do_viz', 'var'); do_viz = false; end

if ~isempty(cameraPars)
    im = undistortImage(im, cameraPars);
end

numPts = length(pointsXYZ);
m = P * [pointsXYZ; ones(1,numPts)]; 
m = m(1:2,:) ./ m(3,:); 

col = zeros(3,numPts);
winsz = 1;
for k = 1:numPts
    kint  = round(m(:,k));   
    i   = kint(2)-winsz:kint(2)+winsz;
    j   = kint(1)-winsz:kint(1)+winsz;
    if any(i>size(im,1)) || any(j>size(im,2))
        continue
    end
    [ii, jj] = meshgrid(i,j);
    for  rgb = 1:3
        colNum  = double(im(i(:),j(:),rgb));
        fcol    = scatteredInterpolant(jj(:),ii(:),colNum(:), 'linear');
        col(rgb,k) = fcol(m(1,k), m(2,k));
    end 
end
col = uint8(col);

if do_viz == true
    figure; imshow(im); hold on;  axis on; 
    scatter(m(1,:), m(2,:), 20, double(col(:, :))'/255,  'filled' , 'MarkerEdgeColor', 	[0 0 0])
end

end