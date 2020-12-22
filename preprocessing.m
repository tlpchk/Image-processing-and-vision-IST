%% cam_params preprocessing
cam_params = load('materials/calib_asus.mat');
cam_params = struct( ...
    'Kdepth', cam_params.Depth_cam.K, ...
    'Krgb', cam_params.RGB_cam.K, ...
    'R', cam_params.R_d_to_rgb,...
    'T', cam_params.T_d_to_rgb);
save('cam_params.mat', 'cam_params');


%% imgseq_newpiv2 preprocessing
n_images = 9;
dataset_path = './datasets/newpiv2/';
rgb_filename_format = strcat(dataset_path,'rgb_image_%d.png');
depth_filename_format_mat = strcat(dataset_path,'depth_%d.mat');
depth_filename_format = strcat(dataset_path,'depth_%d.png');

% Convert depth files from .mat to .png

for i=1:n_images
    d_struct=load(sprintf(depth_filename_format_mat,i));
    imwrite(d_struct.depth_array,sprintf(depth_filename_format,i), 'Mode', 'lossless');
end

imgseq = (struct('rgb', '', 'depth', '')); % init array of structures
for i=1:n_images
    imgseq(i) = struct( ...
        'rgb', sprintf(rgb_filename_format,i), ...
        'depth', sprintf(depth_filename_format,i));
end

save('imgseq_newpiv2', 'imgseq');

%% imseq_short preprocessing
dataset_path = './datasets/short/';
rgb_filename_format = strcat(dataset_path,'rgb_image_%d.png');
depth_filename_format_mat = strcat(dataset_path,'depth_%d.mat');
depth_filename_format = strcat(dataset_path,'depth_%d.png');

% Convert depth files from .mat to .png

for i=12:15
    d_struct=load(sprintf(depth_filename_format_mat,i));
    imwrite(d_struct.depth_array,sprintf(depth_filename_format,i), 'Mode', 'lossless');
end

imgseq = (struct('rgb', '', 'depth', '')); % init array of structures
for i=12:15
    imgseq(i-11) = struct( ...
        'rgb', sprintf(rgb_filename_format,i), ...
        'depth', sprintf(depth_filename_format,i));
end

save('imgseq_short', 'imgseq');

%% imgseq_hobbesquiet preprocessing
dataset_path = './datasets/hobbesquiet/';
rgb_filename_format = strcat(dataset_path,'rgb_%04d.jpg');
depth_filename_format = strcat(dataset_path,'depth_%04d.png');

imgseq = (struct('rgb', '', 'depth', '')); % init array of structures
for i=1:39
    imgseq(i) = struct( ...
        'rgb', sprintf(rgb_filename_format,i-1), ...
        'depth', sprintf(depth_filename_format,i-1));
end

save('imgseq_hobbesquiet', 'imgseq');
%% imgseq_midair_png preprocessing

% dataset_path = './datasets/midair_png/';
% rgb_filename_format = strcat(dataset_path,'color_left/trajectory_5002/00000%d.JPEG');
% depth_filename_format_mat = strcat(dataset_path,'depth_normal/00000%d.mat');
% depth_filename_format = strcat(dataset_path,'depth_normal/00000%d.PNG');