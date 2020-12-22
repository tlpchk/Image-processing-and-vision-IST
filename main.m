imgseq = load('imgseq_newpiv2.mat').imgseq(1:2);
cam_params = load('cam_params.mat').cam_params;
[transforms, XYZ, RGB] = rigid_transforms(imgseq, 1, cam_params);

figure(1); clf ;
showPointCloud(pointCloud(XYZ, 'Color', RGB));
line([1 0; 1 0.5],[1 0; 2 0.3], [1 0; 1.5 1]);
view([0,0,-1]);
drawnow;
