function [R1_to_2, T1_to_2] = rigid_transform(image1, image2, xyz1, xyz2, cam_params)
% rigid_transform - Function that, when receiving the pair of images, produces the matrix
% R and the vector T, which represent the transformation of the 3D points of the camera
% depth of the first image for the second image.
%
%  Arguments: 
%   - image 1: RGB image from first camera;
%   - image 2: RGB image from second camera;
%   - xyz1: 3D points obtained by the first depth camera
%   - xyz2: 3D points obtained by the second depth camera
%
%  Output:
%   - The matrix R and the vector T describe the transformation of the 3D points of the
% depth camera of image1 in the image2 coordinate system.

%% Initialization
if ~exist('vl_sift')
    run('./vlfeat-0.9.21/toolbox/vl_setup');
end
[h, w] = size(image1);
nr_points = 4;
tresh = 0.3;
ransac_iterations = 1000; 
best_inliers = [];

%% SIFT
[f1,d1] = vl_sift(single(rgb2gray(image1)));
[f2,d2] = vl_sift(single(rgb2gray(image2)));
[matches, ~] = vl_ubcmatch(d1, d2);


%% RANSAC
% Collection of 4 pairs of points picked randomly from the matrice of
% matches (with common points between two images) to compute rotation matrice and
% translation vector
for i=1:ransac_iterations
    %% 4 pair of points picked randomly from matrice of matches
    dots = randperm(length(matches),nr_points);
    u1 = f1(1,matches(1,dots));
    v1 = f1(2,matches(1,dots)); 
    u2 = f2(1,matches(2,dots));
    v2 = f2(2,matches(2,dots));

    %% Getting xyz using rgb
    ind_dots1 = sub2ind([h w],uint64(v1),uint64(u1));
    ind_dots2 = sub2ind([h w],uint64(v2),uint64(u2));
    points1 = xyz1(ind_dots1,:);
    points2 = xyz2(ind_dots2,:);
    
    %% Calculation of transformation (R and T)
    [~,~,tr] = procrustes(points2, points1, 'scaling', false,'reflection', false);
    R12 = tr.T;
    T12 = tr.c(1,:)';

    %% Obtain (x,y) pairs from all the points of the matches matrice from both images
    x1 = f1(1,matches(1,:));
    y1 = f1(2,matches(1,:));
    x2 = f2(1,matches(2,:));
    y2 = f2(2,matches(2,:));

    ind1 = sub2ind([h w],uint64(y1),uint64(x1));
    ind2 = sub2ind([h w],uint64(y2),uint64(x2));

    %% Error calculation between points of both images, considering the transformation of points from image 1
    error_eq = xyz2(ind2,:)-((xyz1(ind1,:))*R12+repmat(T12',length(matches),1));
    error = sqrt(sum(error_eq.^2,2));

    %% Finding inliers (indexes) based on the threshold defined previously
    inliers = find(error<tresh);
    if isempty(inliers)
        inliers = [];
    end
    
    % Check if number of inliers are bigger than half of the number of
    % matches and if is bigger than the previous founded inliers then they
    % are updated with new inliers
    if ((length(inliers) >= round(0.5*length(matches))) && (length(inliers) >= length(best_inliers))) % check best_inliers at first iteration
        best_inliers = inliers; % Index of the best inliers founded
    end
end

%% Comparing SIFT mathing with RANSAC and without it
% show_matches(image1, image2, f1, f2, matches, single(best_inliers.'));

%% Selection of all the inliers points of the matrix in order to obtain the rotation matrix and the final translation vector                                                
u1 = f1(1,matches(1,best_inliers));
v1 = f1(2,matches(1,best_inliers));
u2 = f2(1,matches(2,best_inliers));
v2 = f2(2,matches(2,best_inliers));

%% Calculation of final transformation (with best inliers) 
ind1 = sub2ind([h w],uint64(v1),uint64(u1));
ind2 = sub2ind([h w],uint64(v2),uint64(u2));
points1 = xyz1(ind1,:);
points2 = xyz2(ind2,:);

% show_matches_3d(xyz1, xyz2, get_color(xyz1, image1, cam_params), get_color(xyz2, image2, cam_params), points1, points2)

[~,~,tr] = procrustes(points2, points1, 'scaling', false,'reflection', false);
R1_to_2 = tr.T';
T1_to_2 = tr.c(1,:)';

end