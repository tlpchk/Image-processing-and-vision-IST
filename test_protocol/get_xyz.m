function XYZ = get_xyz(depth_array, K_depth)
    [h, w] = size(depth_array);
    [x, y] = meshgrid(1:w, 1:h);
    z = double(depth_array) * 0.001; % to meters
    [x, y, z] = deal(x(:), y(:), z(:));
    XYZ = [z.*x, z.*y, z] * inv(K_depth).';
end