% TODO:
% k - is not used
% max_n_points not used

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
    for i=2:length(imgseq)
        rgb = imread(imgseq(i).rgb);
        depth = imread(imgseq(i).depth);
        xyz = get_xyz(depth, cam_params.Kdepth);
        
        [R, T] = rigid_transform(rgb, rgb_prev, xyz, xyz_prev, cam_params); 
       
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
    
    
    % k frame is world coordinate system
    for i = 1:length(imgseq)
        K1 = [FromCam2W(i).R FromCam2W(i).T; 0 0 0 1];
        K2 = [FromCam2W(k).R FromCam2W(k).T; 0 0 0 1];
        K = inv(K2) * K1;
        FromCam2W(i).R = K(1:3, 1:3);
        FromCam2W(i).T = K(1:3, 4);
    end
    
    best_XYZ = XYZ;
    best_RGB = RGB;
    for i=(0.1:-0.005:0.001)
        pc = pointCloud(XYZ, 'Color', RGB);
        pc = pcdownsample(pc,'gridAverage',i);
        if size(pc.Location,1) > max_n_points
            XYZ = best_XYZ;
            RGB = best_RGB;
            break
        end
        best_XYZ = pc.Location;
        best_RGB = pc.Color;
    end

end