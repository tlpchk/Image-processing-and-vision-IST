imgseq = load('imgseq_newpiv2.mat');
cam_params = load('cam_params.mat');
imgseq = imgseq.imgseq;
cam_params = cam_params.cam_params;
[transforms, XYZ, RGB] = rigid_transforms(imgseq, 1, cam_params, 480*640*5);

figure(1); clf ;
pc = pointCloud(XYZ, 'Color', RGB);
showPointCloud( pointCloud(XYZ, 'Color', RGB) );
view([0,0,-1]);
drawnow;

