function colors = get_color(xyz, rgb, cam_params)
    h = size(rgb, 1);
    w = size(rgb, 2);
    
    xyz1 = [xyz, ones(h*w,1)];
    
    uvw = cam_params.Krgb * [cam_params.R, cam_params.T] * xyz1.';
    u = uvw(1,:) ./ uvw(3,:);
    v = uvw(2,:) ./ uvw(3,:);
    
    [u,v] = deal(round(u), round(v));
    u = max(min(u,w),1);
    v = max(min(v,h),1);
    
    ids = sub2ind(size(rgb), v, u);
    colors_aux = reshape(rgb,h*w,3);
    colors = colors_aux(ids,:);
    colors(xyz(:,1) == 0 & xyz(:,2) == 0 & xyz(:,3) == 0,:) = 0;
    colors = uint8(colors);
end



