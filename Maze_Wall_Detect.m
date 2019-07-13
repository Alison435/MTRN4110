% This file uses image processing techniques to detect the presence of a
% walls. It requires an image to be read and then using image processing
% techniques to detect walls.

% Requires Image Processing Toolbox
close all; clear all;

% [mazeImage,user_canceled]=imgetfile;
mazeImage = imread('Maze & Robot Image_07_09_16_19_34.jpg');
% mazeImage =  imread('Maze & Robot Image_07_10_14_56_03.jpg');
% mazeImage = imread('Maze & Robot Image_07_03_12_35_32.jpg');
% mazeImage = imread('Maze & Robot Image_07_03_13_50_49.jpg');
% mazeImage = imread('Maze & Robot Image_07_03_14_05_02.jpg');


% mazeCapture = imcrop(mazeImage,[110 100 1650 905]);
% mazeCapture = imcrop(mazeImage,[50 50 1780 970]);%crops top part of side wall
% mazeCapture = imcrop(mazeImage,[135 110 1621 873]);%crops entire side wall
mazeCapture = imcrop(mazeImage,[135 110 1620 855]);%crops entire side wall

mazeGRAY = rgb2gray(mazeCapture); % used when image is cropped
% mazeGRAY = rgb2gray(mazeImage); % used when image is not cropped

se = strel('square',1);
se2 = strel('line',2,20);
mazeGRAY22=imgaussfilt(mazeGRAY,1.5);
mazeGRAY2=imdilate(mazeGRAY22,se2);

mazeIMB = imbinarize(mazeGRAY2,'adaptive','ForegroundPolarity','dark','Sensitivity',0.3);
% mazeIMB = imbinarize(mazeGRAY2,'adaptive','ForegroundPolarity','dark','Sensitivity',0.2);
filteredIMB=padarray(mazeIMB,[100 100], 'replicate','both'); %replicates edge values for 100 pixels in both directions
% figure;imshow(filteredIMB); hold on

mazeERODE=imerode(filteredIMB,se);
mazeEDGE4 = edge(mazeERODE,'Canny',0.4,['vertical']);
% mazeEDGE4 = edge(mazeIMB,'Canny',0.4,['vertical']);

BW2 = bwareafilt(mazeEDGE4,[75 400],4);
% BW2 = bwareafilt(mazeEDGE4, 35, 'largest');
mazeEDGE5 = edge(mazeIMB,'Sobel');

% imshow(~mazeIMB); hold on;
% figure;imshow(mazeCapture);hold on;
mazePAD=padarray(mazeGRAY,[100 100], 'replicate','both');
figure;imshow(mazePAD);hold on;
% figure;imshow(mazeImage);hold on;

visboundaries(BW2,'Color','r');

