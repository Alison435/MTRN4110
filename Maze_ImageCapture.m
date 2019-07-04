% Capturing image from 'webcam' above maze - Phase B Vision
close all;clear;
% all;clc;
% Modified from MTRN4230 Sample Code
% Required Packages:
% 1. Image Acquisition Toolbox 
% 2. Image Acquisition Toolbox Support Package
% 3. MATLAB Support Package for USB Webcams 

% Create Camera Object with discovered name of webcam
% NOTE: change to the name of the webcam detected when 'webcamlist' command
% is run in 
maze_image = webcam('Logitech BRIO');
maze_image.Resolution = ('1920x1080');
maze_image.Exposure=-5;
maze_image.Contrast= 80 ;
maze_image.Saturation=100;
maze_image.WhiteBalance=3000;
maze_image
% Preview image from webcam
capture_image(maze_image,'Maze & Robot Image');
clear('maze_image');

% Image capture function - saves image in current directory
function capture_image (vid,name)
 	mazeCapture = snapshot(vid);
    imshow(mazeCapture);
    imwrite(mazeCapture, [name, datestr(datetime('now'),'_mm_dd_HH_MM_SS'), '.jpg']);
    disp([name ' captured']);
end

        