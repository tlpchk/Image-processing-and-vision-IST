%% -- CRIAR LISTA IMAGENS
%DATA DIRECTORY - PATH (with / at the end)
base_data_dir=[pwd '/imgstest/'];
d1=dir([base_data_dir 'depth*']);
r1=dir([base_data_dir 'rgb*']);
if exist('im1'),
    clear im1;
end
for i=1:length(d1),
    im1(i).rgb=[base_data_dir r1(i).name];
    im1(i).depth=[base_data_dir d1(i).name];
end
%load calibration data
load cameraparametersAsus;
maxnpts=500000;
%%
[t2w, xyz, rgb] = rigid_transforms( im1,1,cam_params,maxnpts);
%%
if numel(xyz)>3*maxnpts || numel(rgb)>3*maxnpts,
    error(' too many points returned');
else,
    pc=pointCloud(xyz, 'Color',rgb);
end
figure(1);
pcshow(pc);
figure(2);
%replace pointsbig for the point cloud of each image and check that the
%transformations are ok
load auxfile;
for i=2:length(t2w),
    xyzc=pointsbig{i,1}.Location;
    xyzw=[t2w(i).R t2w(i).T]*[xyzc';ones(1,length(xyzc))];
    pcshow(pointCloud(xyzw','Color',pointsbig{i,2}));
    pause;
end



