function show_matches_3d(xyz1, xyz2, rgb1, rgb2, k_points1, k_points2)
    figure(2); clf ;
    showPointCloud(pointCloud([xyz1; xyz2], 'Color', [rgb1; rgb2]));
    line([k_points1(:,1) k_points2(:,1)],...
        [k_points1(:,2) k_points2(:,2)],...
        [k_points1(:,3) k_points2(:,3)]);
    view([0,0,-1]);
    drawnow;
end