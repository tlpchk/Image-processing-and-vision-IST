function [FromCam2W, XYZ, RGB] = rigid_transforms(imgseq, k, cam_params, max_n_points) 
    FromCam2W = (struct('R','','T','')); % initialization
    
    for i=1:length(imgseq)
        FromCam2W(i).R = eye(3);
        FromCam2W(i).T = zeros(3,1);
    end
    
    rgb1 = imread(imgseq(k).rgb);
    depth1 = imread(imgseq(k).depth);
    xyz1 = get_xyz(depth1, cam_params.Kdepth);
    color = get_color(xyz1, rgb1, cam_params);
    XYZ = (xyz1);
    RGB = (color);
    
    for i=[k-1:-1:1, k+1:length(imgseq)]
        if i == k-1 || i == k+1
           rgb1 = imread(imgseq(k).rgb);
           depth1 = imread(imgseq(k).depth);
           xyz1 = get_xyz(depth1, cam_params.Kdepth);
        end
        
        rgb2 = imread(imgseq(i).rgb);
        depth2 = imread(imgseq(i).depth);
        xyz2 = get_xyz(depth2, cam_params.Kdepth);
        
        [R, T] = rigid_transform(rgb2, rgb1, xyz2, xyz1); 
        
        if i < k
            FromCam2W(i).R = FromCam2W(i+1).R * R ;
            FromCam2W(i).T = (FromCam2W(i+1).R * T) + FromCam2W(i+1).T;
        else
            FromCam2W(i).R = FromCam2W(i-1).R * R ;
            FromCam2W(i).T = (FromCam2W(i-1).R * T) + FromCam2W(i-1).T;
        end
       
        xyz2_transformed = xyz2 * FromCam2W(i).R' + repmat(T', length(xyz2),1);
        colors2_transformed = get_color(xyz2, rgb2, cam_params);
        
        XYZ = [XYZ ; xyz2_transformed];
        RGB = [RGB ; colors2_transformed];
    
        rgb1 = rgb2;
        xyz1 = xyz2;
    end
end