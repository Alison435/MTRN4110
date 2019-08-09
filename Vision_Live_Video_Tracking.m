close all; clear all; clc;  

fig1 =figure(1);
axe1 = axes ();
axe1.Parent = fig1;

%% Comment this section out when using a pre-recorded video
vid = videoinput('winvideo', 1, 'YUY2_1280x720')
% vid = videoinput('winvideo', 2, 'YUY2_1280x720');
set(vid, 'ReturnedColorspace', 'rgb');
vid.SelectedSourceName = 'input1';
src = getselectedsource(vid);
% src.FrameRate = 30;
% src.Contrast = 80;
% src.Saturation = 100;
% src.Gamma = 30;
% src.Hue = 50;
% src.Exposure=-5;
% src.Contrast= 80 ;
% src.Saturation=0;%100;
% src.WhiteBalance=3000;
% src.BacklightCompensation='off';
% src.Exposure=-5;
% src.Contrast= 40 ;
% src.Saturation=0;%100;
% src.WhiteBalance=3000;
% src.Gain=100;
% src.Focus=0;
src2=src;
% Enlarge figure to full screen.
set(gcf, 'units','normalized','outerposition',[0 0 1 1]);

%%
% % open(vid)
% mov = VideoWriter('maze.avi','Motion JPEG AVI');   % Create a new AVI file
% mov.Quality=50;
% mov.FrameRate=5;
% mov.Height;
% mov.Width;
% open(mov)
%%
iFrame = 1;
while (1)
MazeRGB=getsnapshot(vid); %to be used when using webcam to take video
imshow(MazeRGB); hold on


[blueBW,maskedBlueImage] = blueMask(MazeRGB);
blueBox = regionprops(maskedBlueImage, 'Centroid', 'BoundingBox');
frontCentroids = cat(1,blueBox.Centroid);

[greenBW,maskedGreenImage] = greenMask(MazeRGB);
greenBox = regionprops(maskedGreenImage, 'Centroid', 'BoundingBox');
backCentroids = cat(1,greenBox.Centroid);
    if isempty(blueBW) < 1
        FrontCentre = [nanmean(frontCentroids(:,1)),nanmean(frontCentroids(:,2))];
        BackCentre = [nanmean(backCentroids(:,1)),nanmean(backCentroids(:,2))];
        botCentre=[(FrontCentre(1)+BackCentre(1))/2,(FrontCentre(2)+BackCentre(2))/2];
        
        hold on
        RobotFront(iFrame,:)= [FrontCentre(1) FrontCentre(2)];
        plot(RobotFront(:,1),RobotFront(:,2),'r*-');%plots trajectory or car
        viscircles(FrontCentre,80,'Color','c')
        hold on
        iFrame = iFrame + 1;
    end

end
   
function [blueBW,maskedBlueImage] = blueMask(RGB)

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.603;
channel1Max = 0.639;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.646;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.388;
channel3Max = 0.835;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
blueBW = sliderBW;

% Initialize output masked image based on input image.
maskedBlueImage = RGB;

% Set background pixels where BW is false to zero.
maskedBlueImage(repmat(~blueBW,[1 1 3])) = 0;

end

function [greenBW,maskedGreenImage] = greenMask(RGB)

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.474;
channel1Max = 0.518;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.669;
channel2Max = 1.000;


% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.533;
channel3Max = 0.749;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) | (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
greenBW = sliderBW;

% Initialize output masked image based on input image.
maskedGreenImage = RGB;

% Set background pixels where BW is false to zero.
maskedGreenImage(repmat(~greenBW,[1 1 3])) = 0;

end

