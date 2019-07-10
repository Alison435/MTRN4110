% Capturing image from 'webcam' above maze - Phase B Vision
%maze detection
close all; clear all;
%%
% Create Camera Object with discovered name of webcam
% NOTE: change to the name of the webcam detected when 'webcamlist' command
% is run in 
maze_image = webcam('Logitech BRIO');
% maze_image = webcam('USB2.0 HD UVC WebCam');
maze_image.Resolution = ('1920x1080');
% maze_image.Exposure=-5;
% maze_image.Contrast= 80 ;
% maze_image.Saturation=100;
% maze_image.WhiteBalance=3000;
% maze_image;
% Preview image from webcam
% asdf=capture_image(maze_image,'Maze & Robot Image');
% clear('maze_image');
mazeCaptureRaw = snapshot(maze_image);
imshow(mazeCaptureRaw);
% imwrite(mazeCapture, [mazeCapture, datestr(datetime('now'),'_mm_dd_HH_MM_SS'), '.jpg']);
% disp([mazeCapture ' captured']);
mazeCapture = imcrop(mazeCaptureRaw,[110 100 1650 905]); % crops image to remove the side walls
mazeGRAY = rgb2gray(mazeCapture);
se = strel('square',1);
se2 = strel('line',2,20);
mazeGRAY22=imgaussfilt(mazeGRAY,1.5);
mazeGRAY2=imdilate(mazeGRAY22,se2);

mazeIMB = imbinarize(mazeGRAY2,'adaptive','ForegroundPolarity','dark','Sensitivity',0.35);
% mazeIMB = imbinarize(mazeGRAY,'adaptive');
% mazeIMB = imbinarize(mazeGRAY,'global');
% imshow(~mazeIMB);

mazeERODE=imerode(mazeGRAY2,se);
mazeEDGE4 = edge(mazeERODE,'Canny',0.4,['vertical']);
% BW2 = bwareafilt(mazeEDGE4,[80 1000],4);
BW2 = bwareafilt(mazeEDGE4, 385, 'largest');
% mazeEDGE4 = edge(mazeIMB,'Canny',0.4,['vertical']);
% imshow(mazeEDGE4);
mazeEDGE5 = edge(mazeIMB,'Sobel');

% imshow(~mazeIMB); hold on;
imshow(mazeCapture);hold on;
visboundaries(BW2,'Color','r');
title('BWarea');
hold off;
