%%
run('./vlfeat-0.9.21/toolbox/vl_setup')

%%
im1_rgb = imread('frame.jpg');
im2_rgb = imread('template.png');

im1 = single(im2gray(im1_rgb));
im2 = single(im2gray(im2_rgb));

[h1, w1] = deal(size(im1,1),size(im1,2));
[h2, w2] = deal(size(im2,1),size(im2,2));

[f1,d1] = vl_sift(im1,'PeakThresh', 4);
[f2,d2] = vl_sift(im2);
[matches, scores] = vl_ubcmatch(d1,d2);

%%
numMatches = size(matches,2);
distance_tresh = 5;

clear H score ok_dict ;
global ok X1 X2;

X1 = f1(1:2,matches(1,:)) ; X1(3,:) = 1 ;
X2 = f2(1:2,matches(2,:)) ; X2(3,:) = 1 ;

for t = 1:1000
  % estimate homography
    subset = vl_colsubset(1:numMatches, 4) ;
    H{t} = dlt(X1(:,subset),X2(:,subset));
    
    % score homography
    X2_ = H{t} * X1 ;
    du = X2_(1,:)./X2_(3,:) - X2(1,:)./X2(3,:) ;
    dv = X2_(2,:)./X2_(3,:) - X2(2,:)./X2(3,:) ;
    ok_dict{t} = (du.*du + dv.*dv) < distance_tresh*distance_tresh ;
    score(t) = sum(ok_dict{t}) ;
end

[score, best] = max(score) ;
H = H{best} ;
ok = ok_dict{best} ;

H = H / H(3,3);
opts = optimset('PlotFcns',@optimplotfval);
H(1:8) = fminsearch(@residual, H(1:8)', opts);


%%
% Show matches

dh1 = max(size(im2,1)-size(im1,1),0) ;
dh2 = max(size(im1,1)-size(im2,1),0) ;

figure(1) ; clf ;
subplot(1,2,1) ;
imagesc([padarray(im1,dh1,'post') padarray(im2,dh2,'post')]) ;

o = size(im1,2) ;
line([f1(1,matches(1,:));f2(1,matches(2,:))+o], ...
     [f1(2,matches(1,:));f2(2,matches(2,:))]) ;
title(sprintf('%d tentative matches', numMatches)) ;
axis image off ;

subplot(1,2,2) ;
imagesc([padarray(im1,dh1,'post') padarray(im2,dh2,'post')]) ;
o = size(im1,2) ;
line([f1(1,matches(1,ok));f2(1,matches(2,ok))+o], ...
     [f1(2,matches(1,ok));f2(2,matches(2,ok))]) ;
title(sprintf('%d (%.2f%%) inliner matches out of %d', ...
              sum(ok), ...
              100*sum(ok)/numMatches, ...
              numMatches)) ;
axis image off ;

drawnow ;

%% draw predicted conture of object
corners = inv(H) * [1 1 w2 w2;...
                    1 h2 h2 1;...
                    1 1 1 1];
corners_x = round(corners(1,:) ./ corners(3,:));
corners_y = round(corners(2,:) ./ corners(3,:));
corners = [corners_x;corners_y];

im1_corners = insertShape(im1_rgb, 'Polygon', corners(:).','LineWidth', 15);
figure;
imshow(im1_corners);


%% result
tform = projective2d(H.');

imOut = imwarp(im1_rgb, tform, 'OutputView', imref2d(size(im2_rgb)));
figure;
imshow(imOut);

%% 
function err = residual(H)
    global X1 X2 ok;
    u = H(1) * X1(1,ok) + H(4) * X1(2,ok) + H(7) ;
    v = H(2) * X1(1,ok) + H(5) * X1(2,ok) + H(8) ;
    d = H(3) * X1(1,ok) + H(6) * X1(2,ok) + 1 ;
    du = X2(1,ok) - u ./ d ;
    dv = X2(2,ok) - v ./ d ;
    err = sum(du.*du + dv.*dv) ;
end
