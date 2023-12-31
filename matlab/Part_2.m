% Load light field data
load('lightField.mat');

%% Part 1 - lightfield.mat

% create parameters as described (to adjust manually)
sensor_width = .016;  % in mm!
num_pixels = 450;

rays_x = rays(1, :); %load rays x positions into a row vector
rays_y = rays(3, :); %load rays y positions into a row vector


[image, x, y] = rays2img(rays_x, rays_y, sensor_width, num_pixels);
%the x and y output by the function don't really matter, they'll always
%just be half of the sensor width so not terribly important, in the future
%we'll just sub with a ~ to ignore their outputs....

figure;
imshow(image);
title('Image of Rays After Refining Sensor Width and Pixel Count')

propagation_distances = [0.1, 0.2, 0.4]; %create different propagation distances to simulate

for d = propagation_distances
    raysXProp = rays_x + rays(2, :) * d; % Propagate in xz plane
    raysYProp = rays_y + rays(4, :) * d; % Propagate in yz plane
    [image, ~, ~] = rays2img(raysXProp, raysYProp, sensor_width, num_pixels);
    figure;
    imshow(image); 
    title(['Propagation Distance of ', num2str(d), 'm']);
end

%% Part 3 - Create an Image

f = 0.149; %focal length (meters)
d2 = 0.225; %distance lens --> sensor
sensor_width = 0.008; %sensor width
num_pixels = 650;

lensMatrix = [1 0 0 0; %build the matrix for passing through the lens (basically just adjust the slopes of the rays that pass through)
            -1/f 1 0 0;
              0 0 1 0;
              0 0 -1/f 1];

propagationMatrix = [1 d2 0 0; %build the propagation matrix as outlined done in previous part
                     0 1 0 0;
                     0 0 1 d2;
                     0 0 0 1];

raysTransformed = propagationMatrix * lensMatrix * [rays(1,:); rays(2,:); rays(3,:); rays(4,:)]; %generate the final rays vectors

rays_x_after = raysTransformed(1,:); %get the x positions of all the rays
rays_y_after = raysTransformed(3,:); %get the y positions of all the rays

[image, ~, ~] = rays2img(rays_x_after, rays_y_after, sensor_width, num_pixels); %call rays2img to yield the final image


figure;
imshow(fliplr(image)); %display the final (flipped) image in a new figure
title('Image formed by the optical system');


%% Multiple Lens Experimentation

f1 = 0.149; 
d2 = 0.232;


f2 = 0.06; 
d3 = 0.015; 

sensor_width = 0.008; 
num_pixels = 650;

lensMatrix1 = [1 0 0 0;
              -1/f1 1 0 0;
               0 0 1 0;
               0 0 -1/f1 1];

lensMatrix2 = [1 0 0 0; %create new matrix for the different focal length of the second lens
              -1/f2 1 0 0;
               0 0 1 0;
               0 0 -1/f2 1];

propagationMatrix1 = [1 d2 0 0;
                      0 1 0 0;
                      0 0 1 d2;
                      0 0 0 1];

propagationMatrix2 = [1 d3 0 0; %create new propagation matrix for the different lens to sensor distance
                      0 1 0 0;
                      0 0 1 d3;
                      0 0 0 1];

raysTransformed = propagationMatrix2 * lensMatrix2 * propagationMatrix1 * lensMatrix1 * [rays(1,:); rays(2,:); rays(3,:); rays(4,:)];
%repeat the old math but just add the matrix-matrix product of the two new matrices to include the second lens


rays_x_after = raysTransformed(1,:);
rays_y_after = raysTransformed(3,:);


[image, ~, ~] = rays2img(rays_x_after, rays_y_after, sensor_width, num_pixels);


figure;
imshow(fliplr(image)); %show the flipped image using fliplr function
title('Image formed by the dual-lens optical system');

%% Third Lens Experimentation

f1 = 0.149; 
d2 = 0.232;


f2 = 0.06; 
d3 = 0.015; 


f3 = 0.075;
d4 = 0.002;


lensMatrix1 = [1 0 0 0;
              -1/f1 1 0 0;
               0 0 1 0;
               0 0 -1/f1 1];

lensMatrix2 = [1 0 0 0;
              -1/f2 1 0 0;
               0 0 1 0;
               0 0 -1/f2 1];

lensMatrix3 = [1 0 0 0; %create third matrix for the different focal length of the third lens
              -1/f3 1 0 0;
               0 0 1 0;
               0 0 -1/f3 1];

propagationMatrix1 = [1 d2 0 0;
                      0 1 0 0;
                      0 0 1 d2;
                      0 0 0 1];

propagationMatrix2 = [1 d3 0 0;
                      0 1 0 0;
                      0 0 1 d3;
                      0 0 0 1];

propagationMatrix3 = [1 d4 0 0; %create new propagatoin matrix for the different distance between the third lens and the sensor
                      0 1 0 0;
                      0 0 1 d4;
                      0 0 0 1];


sensor_width = 0.008; %redefine sensor width. kind of redundant i guess
num_pixels = 650; 


raysTransformed = propagationMatrix3 * lensMatrix3 * propagationMatrix2 * lensMatrix2 * propagationMatrix1 * lensMatrix1 * [rays(1,:); rays(2,:); rays(3,:); rays(4,:)];


rays_x_after = raysTransformed(1,:);
rays_y_after = raysTransformed(3,:);


[image, ~, ~] = rays2img(rays_x_after, rays_y_after, sensor_width, num_pixels);

figure;
imshow(fliplr(image));
title('Image formed by the triple-lens optical system');


function [img,x,y] = rays2img(rays_x,rays_y,width,Npixels)
% rays2img - Simulates the operation of a camera sensor, where each pixel
% simply collects (i.e., counts) all of the rays that intersect it. The
% image sensor is assumed to be square with 100% fill factor (no dead
% areas) and 100% quantum efficiency (each ray intersecting the sensor is
% collected).
%
% inputs:
% rays_x: A 1 x N vector representing the x position of each ray in meters.
% rays_y: A 1 x N vector representing the y position of each ray in meters.
% width: A scalar that specifies the total width of the image sensor in 
%   meters.
% Npixels: A scalar that specifies the number of pixels along one side of 
%   the square image sensor.
%
% outputs:
% img: An Npixels x Npixels matrix representing a grayscale image captured 
%   by an image sensor with a total Npixels^2 pixels.
% x: A 1 x 2 vector that specifies the x positions of the left and right 
%   edges of the imaging sensor in meters.
% y: A 1 x 2 vector that specifies the y positions of the bottom and top 
%   edges of the imaging sensor in meters.
%
% Matthew Lew 11/27/2018
% 11/26/2021 - edited to create grayscale images from a rays_x, rays_y
% vectors
% 11/9/2022 - updated to fix axis flipping created by histcounts2()

% eliminate rays that are off screen
onScreen = abs(rays_x)<width/2 & abs(rays_y)<width/2;
x_in = rays_x(onScreen);
y_in = rays_y(onScreen);

% separate screen into pixels, calculate coordinates of each pixel's edges
mPerPx = width/Npixels;
Xedges = ((1:Npixels+1)-(1+Npixels+1)/2)*mPerPx;
Yedges = ((1:Npixels+1)-(1+Npixels+1)/2)*mPerPx;

% count rays at each pixel within the image
img = histcounts2(y_in,x_in,Yedges,Xedges);    % histcounts2 for some reason assigns x to rows, y to columns


% rescale img to uint8 dynamic range
img = uint8(round(img/max(img(:)) * 255));
x = Xedges([1 end]);
y = Yedges([1 end]);

% figure;
% image(x_edges([1 end]),y_edges([1 end]),img); axis image xy;
end