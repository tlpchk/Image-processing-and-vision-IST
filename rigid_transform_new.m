function [R1_to_2, T1_to_2] = rigid_transform_new(image1, image2, xyz1, xyz2)
% cam_params = load('materials/calib_asus.mat');
% Kdepth = cam_params.Depth_cam.K;
% 
% image1 = imread('./datasets/newpiv2/rgb_image_2.png');
% depth1 = imread('./datasets/newpiv2/depth_2.png');
% xyz1 = get_xyz(depth1, Kdepth);
% 
% image2 = imread('datasets/newpiv2/rgb_image_3.png');
% depth2 = imread('datasets/newpiv2/depth_3.png');
% xyz2 = get_xyz(depth2, Kdepth);

%% Initialization
if ~exist('vl_sift')
    run('./vlfeat-0.9.21/toolbox/vl_setup');
end
[h, w] = size(image1);
nr_points = 4;
tresh = 0.05;
ransac_iterations = 2000; 

%% SIFT
[f1,d1] = vl_sift(single(rgb2gray(image1)));
[f2,d2] = vl_sift(single(rgb2gray(image2)));
[matches, ~] = vl_ubcmatch(d1, d2);

% index=find(scores<20000);
% ok = true(size(matches,2),1);
% ok = ok(index');
% show_matches(image1, image2, f1, f2, matches, ok)

%%
% Filters matches for scores (L2 norm) that are bigger than 20.000 (big
% distances)
% index=find(scores<20000);
% matches=matches(:,index');

%% RANSAC
% Collection of 4 pairs of points picked randomly from the matrice of
% matches (with common points between two images) to compute rotation matrice and
% translation vector
error_trace = [];
best_inliers = [];
best_error = Inf;

show_matches(image1, image2, f1, f2, matches, 1:length(matches));

for i=1:ransac_iterations
    % 4 pair of points picked randomly from matrice of matches
    dots = randperm(length(matches),nr_points);
  
    u1 = f1(1,matches(1,dots));
    v1 = f1(2,matches(1,dots)); 
    u2 = f2(1,matches(2,dots));
    v2 = f2(2,matches(2,dots));

    % Calculation of the centroids base on the 4 pair of points
    ind_dots1 = sub2ind([h w],uint64(v1),uint64(u1));
    ind_dots2 = sub2ind([h w],uint64(v2),uint64(u2));
    points1 = xyz1(ind_dots1,:);
    points2 = xyz2(ind_dots2,:);
    
    [~,~,tr] = procrustes(points2, points1, 'scaling', false,'reflection', false);
    
    % Obtain (x,y) pairs from all the points of the matches matrice from both images
    x1 = f1(1,matches(1,:));
    y1 = f1(2,matches(1,:));
    x2 = f2(1,matches(2,:));
    y2 = f2(2,matches(2,:));

    ind1 = sub2ind([h w],uint64(y1),uint64(x1));
    ind2 = sub2ind([h w],uint64(y2),uint64(x2));

    % Error calculation between points of both images, considering the transformation of points from image 1
    T = repmat(tr.c(1,:),length(matches),1);
    R = tr.T;
    error_eq = xyz2(ind2,:)-(xyz1(ind1,:)*R + T);
    error = sqrt(sum(error_eq.^2,2));
   
    % Finding inliers (indexes) based on the threshold defined previously
    inliers = find(error<tresh);
    if isempty(inliers)
        inliers = [];
    end
    error = error(inliers);
    
    % Check if number of inliers are bigger than half of the number of
    % matches and if is bigger than the previous founded inliers then they
    % are updated with new inliers
    if mean(error) < best_error && length(inliers) >= 0.25*length(matches) % check best_inliers at first iteration
        error_trace = [error_trace, mean(error)];
        best_error = mean(error);
        best_inliers = inliers; % Index of the best inliers founded
    end
end
% figure;
% plot(1:length(error_trace), error_trace);
% show_matches(image1, image2, f1, f2, matches, single(best_inliers.'));
%% Selection of all the inliers points of the matrix in order to obtain the rotation matrix and the final translation vector                                                
u1 = f1(1,matches(1,best_inliers));
v1 = f1(2,matches(1,best_inliers));
u2 = f2(1,matches(2,best_inliers));
v2 = f2(2,matches(2,best_inliers));

ind1 = sub2ind([h w],uint64(v1),uint64(u1));
ind2 = sub2ind([h w],uint64(v2),uint64(u2));
points1 = xyz1(ind1,:);
points2 = xyz2(ind2,:);

[~,~,tr] = procrustes(points2, points1, 'scaling', false,'reflection', false);

T = repmat(tr.c(1,:), length(best_inliers),1);
R = tr.T;
error_eq = xyz2(ind2,:)-(xyz1(ind1,:)*R + T);
mean(sqrt(sum(error_eq.^2,2)))

R1_to_2 = tr.T;
T1_to_2 = tr.c(1,:).';



end
