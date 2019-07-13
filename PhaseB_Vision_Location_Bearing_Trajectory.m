%%This code reads in a video and locates the location of the robot by
%%matchingSURFfeatures and determines the bearing and tracks trajectory
%%It also records a video of the entire process

close all; clear all;
% clc;

%The code requires a sample image of the robot to match the video frame
RobotRGB=imread('RobotPic3.jpg');
% RobotRGB=imread('RobotPic5.jpg');
RobotGRAY=rgb2gray(RobotRGB);
RobotPoints=detectSURFFeatures(RobotGRAY);

%warning('off','all'); %.... diable warining msg ...;
fig1 =figure(1);
axe1 = axes ();
axe1.Parent = fig1;
%% Comment this section out when using a live video
% vid = VideoReader('WIN_20190709_16_43_09_Pro.mp4');
% vid = VideoReader('WIN_20190709_16_36_43_Pro.mp4');
vid = VideoReader('WIN_20190709_16_35_42_Pro.mp4'); % use 'RobotPic3.jpg'
% vid = VideoReader('WIN_20190703_14_10_53_Pro.mp4');

% figure(1)
% wallDetect(mazeImage);
% hold on;

numberOfFrames = vid.NumberOfFrames;
vidHeight = vid.Height;
vidWidth = vid.Width;
% % Enlarge figure to full screen.
set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
%% Comment this section out when using a pre-recorded video
% vid = videoinput('winvideo', 1, 'YUY2_1280x720')
% vid.SelectedSourceName = 'input1';
% src = getselectedsource(vid);
% src.FrameRate = 5.0000;
% src.Contrast = 80;
% src.Saturation = 100;
% src.Gamma = 30;
% src.Hue = 50;
% % Enlarge figure to full screen.
% set(gcf, 'units','normalized','outerposition',[0 0 1 1]);

%%
% open(vid)
mov = VideoWriter('maze.avi','Motion JPEG AVI');   % Create a new AVI file
mov.Quality=50;
mov.FrameRate=10;
mov.Height;
mov.Width;
open(mov)

for iFrame = 1:10:550
% MazeRGB=getsnapshot(vid); %to be used when using webcam to take video

mazeCapture = read(vid, iFrame);
hold on;

MazeRGB = read(vid, iFrame);  % You would capture a single image from your webcam here
MazeGRAY=rgb2gray(MazeRGB);
% MazeGRAY2 = imcrop(MazeGRAY,[110 100 1650 905]);
MazePoints=detectSURFFeatures(MazeGRAY);

[RobotFeatures, RobotPoints] = extractFeatures(RobotGRAY, RobotPoints);
[MazeFeatures, MazePoints] = extractFeatures(MazeGRAY, MazePoints);

boxPairs = matchFeatures(RobotFeatures, MazeFeatures);

matchedRobotPoints = RobotPoints(boxPairs(:, 1), :);
matchedMazePoints = MazePoints(boxPairs(:, 2), :);   

[tform, inlierRobotPoints, inlierMazePoints] = ...
    estimateGeometricTransform(matchedRobotPoints, matchedMazePoints, 'similarity');
%% Comment this section out when using a live video
imshow(MazeGRAY) 
%% Comment this section out when using a pre-recorded video
%    I=getsnapshot(vid);
%    imshow(I)
%%
hold on;
% 
% 
boxPolygon = [1, 1;...                           % top-left
        size(RobotGRAY, 2), 1;...                 % top-right
        size(RobotGRAY, 2), size(RobotGRAY, 1);... % bottom-right
        1, size(RobotGRAY, 1);...                 % bottom-left
        1, 1];                   % top-left again to close the polygon

% boxPolygon = polyshape([1 size(RobotGRAY, 2) size(RobotGRAY, 2)  1],...
%              [1 1 size(RobotGRAY, 1) size(RobotGRAY, 1)]);
    
newBoxPolygon = transformPointsForward(tform, boxPolygon);
line(newBoxPolygon(:, 1), newBoxPolygon(:, 2), 'Color', 'y');

RobotCentroid(iFrame,:)= [mean(newBoxPolygon(:,1)) mean(newBoxPolygon(:,2))];
% hold off;
plot(RobotCentroid(:,1),RobotCentroid(:,2),'r*');%plots trajectory or car

hold on;

Tinv  = tform.invert.T;

ss = Tinv(2,1);
sc = Tinv(1,1);
% scaleRecovered = sqrt(ss*ss + sc*sc)
theta = -atan2(ss,sc)*180/pi
r=300;%theta=0;
x=RobotCentroid(iFrame,1); y=RobotCentroid(iFrame,2);
[u,v]=pol2cart(-theta*(pi/180),r);% convert polar (theta,r) to cartesian
h = quiver(x,y,-u,-v,'g','linewidth',2);

%% Comment this section out when using a pre-recorded video
%    I=getsnapshot(vid);
%    imshow(I)
%%
FF=getframe(gcf);
% With "VideoWriter" use "writevideo" to add frames to the video
writeVideo(mov,FF);

end
   close(mov)
