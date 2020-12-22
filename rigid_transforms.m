function [FromCam2W, XYZ, RGB] = rigid_transforms(imgseq, k, cam_params, max_n_points) 
    FromCam2W = (struct('R','','T','')); % initialization
    
    for i=1:length(imgseq)
        FromCam2W(i).R = eye(3);
        FromCam2W(i).T = zeros(3,1);
    end
    
    % 1 frame is the world coordinate system
    rgb_world = imread(imgseq(1).rgb);
    depth_world = imread(imgseq(1).depth);
    xyz_world = get_xyz(depth_world, cam_params.Kdepth);
    color_world = get_color(xyz_world, rgb_world, cam_params);
    XYZ = (xyz_world);
    RGB = (color_world);
    
    rgb_prev = rgb_world;
    xyz_prev = xyz_world;
    
    errors = zeros(length(imgseq),1); 
    for i=2:length(imgseq)
        rgb = imread(imgseq(i).rgb);
        depth = imread(imgseq(i).depth);
        xyz = get_xyz(depth, cam_params.Kdepth);
        
        [R, T, errors(i)] = rigid_transform(rgb, rgb_prev, xyz, xyz_prev, cam_params); 
       
        FromCam2W(i).R = FromCam2W(i-1).R * R ;
        FromCam2W(i).T = (FromCam2W(i-1).R * T) + FromCam2W(i-1).T;
        
        K = [FromCam2W(i).R FromCam2W(i).T];
        
        xyz_t = (K * [xyz, ones(length(xyz),1)]')';
        colors = get_color(xyz, rgb, cam_params);
        
        rgb_prev = rgb;
        xyz_prev = xyz;
        
        XYZ = [XYZ ; xyz_t];
        RGB = [RGB ; colors];
    end
    fprintf("Mean distance between inliers in neighbouring pointclouds d=%f\n",mean(errors));
    
    % k frame is world coordinate system
    for i = 1:length(imgseq)
        K1 = [FromCam2W(i).R FromCam2W(i).T; 0 0 0 1];
        K2 = [FromCam2W(k).R FromCam2W(k).T; 0 0 0 1];
        K = inv(K2) * K1;
        FromCam2W(i).R = K(1:3, 1:3);
        FromCam2W(i).T = K(1:3, 4);
    end
    
    pc = pointCloud(XYZ, 'Color', RGB);
    pc = pcdownsample(pc,'random', max_n_points/size(XYZ,1));
    XYZ = pc.Location;
    RGB = pc.Color;
end