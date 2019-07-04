% This file uses image processing techniques to detect the presence of a
% wall
close all; clear all;

[mazeImage,user_canceled]=imgetfile;
% mazeImage = ('Maze & Robot Image_07_03_14_05_18.jpg');
mazeRGB= imread(mazeImage);

% imshow(mazeRGB);

% mazeRGB2 = imerode(mazeRGB);
% imshow(mazeRGB2);

mazeGRAY = rgb2gray(mazeRGB);
mazeGRAY22=imgaussfilt(mazeGRAY,1.5);
se = strel('square',1);
% mazeGRAY222 = imerode(mazeGRAY,se);
mazeGRAY2=imdilate(mazeGRAY22,se);

% mazeIMB = imbinarize(mazeGRAY,'adaptive','ForegroundPolarity','bright','Sensitivity',0.8);
mazeIMB = imbinarize(mazeGRAY2,'adaptive','ForegroundPolarity','dark','Sensitivity',0.35);
% mazeIMB = imbinarize(mazeGRAY,'adaptive');
% mazeIMB = imbinarize(mazeGRAY,'global');
% imshow(~mazeIMB);

se2 = strel('line',2,20);
mazeERODE=imerode(mazeGRAY2,se2);
mazeEDGE4 = edge(mazeERODE,'Canny',0.4,'vertical');
% BW2 = bwareafilt(mazeEDGE4,[80 1000],4);
% mazeEDGE4 = edge(mazeIMB,'Canny',0.4,['vertical']);
mazeEDGE5 = edge(mazeIMB,'Sobel');

% imshow(~mazeIMB); hold on;
imshow(mazeRGB);hold on;
BW2 = bwareafilt(mazeEDGE4, 39, 'largest');
visboundaries(BW2,'Color','r');
title('BWarea');
hold off;
