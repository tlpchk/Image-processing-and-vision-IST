function [R1_to_2, T1_to_2] = rigid_transform(image1, image2, xyz1, xyz2)
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
tresh = 0.4;
ransac_iterations = 1000; 
best_inliers = [];

%% SIFT
[f1,d1] = vl_sift(single(rgb2gray(image1)));
[f2,d2] = vl_sift(single(rgb2gray(image2)));
[matches, scores] = vl_ubcmatch(d1, d2);

% Filters matches for scores (L2 norm) that are bigger than 20.000 (big
% distances)
index=find(scores<20000);
matches=matches(:,index');

%% RANSAC
% Collection of 4 pairs of points picked randomly from the matrice of
% matches (with common points between two images) to compute rotation matrice and
% translation vector
for i=1:ransac_iterations
    %% 4 pair of points picked randomly from matrice of matches
    dots = randperm(length(matches),nr_points);
    u1 = f1(1,matches(1,dots)); % CHECK
    v1 = f1(2,matches(1,dots)); 
    u2 = f2(1,matches(2,dots));
    v2 = f2(2,matches(2,dots));

    %% Calculation of the centroids base on the 4 pair of points
    ind_dots1 = sub2ind([h w],uint64(v1),uint64(u1));
    ind_dots2 = sub2ind([h w],uint64(v2),uint64(u2));
    cent1 = mean(xyz1(ind_dots1,:))';
    cent2 = mean(xyz2(ind_dots2,:))';
    pc1 = xyz1(ind_dots1,:)'-repmat(cent1,1,length(u1));
    pc2 = xyz2(ind_dots2,:)'-repmat(cent2,1,length(u2));

    %% Calculation of transformation (R+T)
    [a b c] = svd(pc2*pc1');
    R12 = a*c';
    T12 = cent2-R12*cent1;

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
        best_nr_inliers = length(inliers); % Biggest number of founded inliers
    end
end
%% Selection of all the inliers points of the matrix in order to obtain the rotation matrix and the final translation vector                                                
u1 = f1(1,matches(1,best_inliers));
v1 = f1(2,matches(1,best_inliers));
u2 = f2(1,matches(2,best_inliers));
v2 = f2(2,matches(2,best_inliers));

%% Centroids calculation based on all inliers
ind1 = sub2ind([h w],uint64(v1),uint64(u1));
ind2 = sub2ind([h w],uint64(v2),uint64(u2));
cent1 = mean(xyz1(ind1,:))';
cent2 = mean(xyz2(ind2,:))';
pc1 = xyz1(ind1,:)'-repmat(cent1,1,length(u1));
pc2 = xyz2(ind2,:)'-repmat(cent2,1,length(u2));

%% Calculation of final transformation (with best inliers) 
[a b c] = svd(pc2*pc1');
R1_to_2 = a*c';
T1_to_2 = cent2-R1_to_2*cent1;
end