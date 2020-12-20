rgb = imread('datasets/newpiv2/rgb_image_1.png');
depth = imread('datasets/newpiv2/depth_1.png');
xyz = get_xyz(depth, cam_params.Kdepth);
colors = get_color(xyz, rgb, cam_params);
point_cloud = pointCloud(xyz);
point_cloud.Color = colors;
showPointCloud(point_cloud);
view([0,0,-1]);