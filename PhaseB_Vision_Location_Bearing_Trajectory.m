% Live video car tracking using detect surf features
% To be updated to detect colour on bot for tracking 17July19

close all; clear all;
% clc;  

%The code requires a sample image of the robot to match the video frame
% RobotRGB=imread('RobotPic3.jpg');
% RobotRGB=imread('RobotPic5.jpg');
RobotRGB=imread('RobotPic6.jpg');
% RobotRGB=imread('RobotPic7.jpg');
% RobotRGB=imread('Arrow1.jpg');
RobotGRAY=rgb2gray(RobotRGB);
RobotPoints=detectSURFFeatures(RobotGRAY);

%warning('off','all'); %.... diable warining msg ...;
fig1 =figure(1);
axe1 = axes ();
axe1.Parent = fig1;

%% Comment this section out when using a live video
% vid = VideoReader('WIN_20190709_16_43_09_Pro.mp4');
% vid = VideoReader('WIN_20190709_16_36_43_Pro.mp4');
% vid = VideoReader('WIN_20190715_11_33_48_Pro.mp4'); % use 'RobotPic3.jpg'
% vid = VideoReader('WIN_20190703_14_10_53_Pro.mp4');
vid = VideoReader('WIN_20190716_15_12_09_Pro.mp4');% use 'RobotPic7.jpg'

numberOfFrames = vid.NumberOfFrames;
vidHeight = vid.Height;
vidWidth = vid.Width;
% % Enlarge figure to full screen.
set(gcf, 'units','normalized','outerposition',[0 0 1 1]);

%%
mov = VideoWriter('maze.avi','Motion JPEG AVI');   % Create a new AVI file
mov.Quality=50;
mov.FrameRate=5;
mov.Height;
mov.Width;
open(mov)

%%
for iFrame = 500:20:1350
% MazeRGB=getsnapshot(vid); %to be used when using webcam to take video

% mazeCapture = read(vid, iFrame);
% hold on;

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

imshow(MazeGRAY) 

hold on;

boxPolygon = [1, 1;...                           % top-left
        size(RobotGRAY, 2), 1;...                 % top-right
        size(RobotGRAY, 2), size(RobotGRAY, 1);... % bottom-right
        1, size(RobotGRAY, 1);...                 % bottom-left
        1, 1];                   % top-left again to close the polygon

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

%%
FF=getframe(gcf);
% With "VideoWriter" use "writevideo" to add frames to the video
writeVideo(mov,FF);

end
   close(mov)
