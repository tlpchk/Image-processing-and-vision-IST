% TODO:
% k - is not used
% max_n_points not used

function [FromCam2W, XYZ, RGB] = new_rigid_transforms(imgseq, k, cam_params, max_n_points) 
    FromCam2W = (struct('R','','T','')); % initialization
    
    for i=1:length(imgseq)
        FromCam2W(i).R = eye(3);
        FromCam2W(i).T = zeros(3,1);
    end
    
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
        
        [R, T] = rigid_transform(rgb, rgb_prev, xyz, xyz_prev); 
       
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
end