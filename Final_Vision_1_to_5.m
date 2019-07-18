%% Code to be used for Vision Parts 1-4
% Uses color threshold to locate bot
close all; clear all;

% maze_image = webcam('Logitech BRIO');
% % maze_image = webcam('USB2.0 HD UVC WebCam');
% maze_image.Resolution = ('1920x1080');
% maze_image.Exposure=-5;
% maze_image.Contrast= 80 ;
% maze_image.Saturation=100;
% maze_image.WhiteBalance=3000;

%% 1. Take an image of the maze and display it on the screen
% mazeCaptureRaw = snapshot(maze_image);
% mazeCaptureRaw = imread('Maze & Robot Image_07_15_13_03_07.jpg');
% mazeCaptureRaw =  imread('Maze & Robot Image_07_10_14_56_03.jpg');
% mazeCaptureRaw = imread('JamesBotWest.jpg');
% mazeCaptureRaw = imread('JamesBotWest2.jpg');
% mazeCaptureRaw = imread('JamesBotNorth.jpg');
mazeCaptureRaw = imread('JamesBotEast.jpg');
% mazeCaptureRaw = imread('JamesBotSouthEast.jpg');
% mazeCaptureRaw = imread('JamesBotNorthWest.jpg');

mazeImageResize=imresize(mazeCaptureRaw,[1080,1920]);
% mazeCapture = imcrop(mazeImageResize,[19 32 1858 1038]);
mazeCapture = imcrop(mazeImageResize,[19 32 1863 1038]);
mazeGRAY=rgb2gray(mazeCapture);
% imshow(mazeCapture);hold on


%% 2. Extract the maze layout and overlay it onto the image in RED colour
% Detects bot and masks it out for edge detection
[blueBW,maskedBlueImage] = blueMask(mazeCapture);
blueBox = regionprops(maskedBlueImage, 'Centroid', 'BoundingBox');
frontCentroids = cat(1,blueBox.Centroid);
FrontCentre = [nanmean(frontCentroids(:,1)),nanmean(frontCentroids(:,2))];

[redBW,maskedRedImage] = redMask(mazeCapture);
redBox = regionprops(maskedRedImage, 'Centroid', 'BoundingBox');
backCentroids = cat(1,redBox.Centroid);
BackCentre = [nanmean(backCentroids(:,1)),nanmean(backCentroids(:,2))];

botCentre=[(FrontCentre(1)+BackCentre(1))/2,(FrontCentre(2)+BackCentre(2))/2];
robotMask=regionfill(mazeGRAY,[botCentre(1)-110 botCentre(1)-110 botCentre(1)+90 botCentre(1)+90  botCentre(1)-110 ],...
                    [botCentre(2)-90 botCentre(2)+80 botCentre(2)+80 botCentre(2)-90 botCentre(2)-90]);
% figure;imshow(robotMask),title('RobotMasked and Position'), hold on
% plot(FrontCentre(1),FrontCentre(2),'b*');
% plot(BackCentre(1),BackCentre(2),'r*');

%%
% Masks cell centres and side faces
robotMask3=regionfill(robotMask,[13 13+185 13+185 13 13], [15 15 15+184 15+184 15]);
robotMask4=regionfill(robotMask3,[210 210+190 210+190 210 210], [13 13 13+190 13+190 13]);
robotMask5=regionfill(robotMask4,[416 416+190 416+190 416 416], [15 15 15+190 15+190 15]);
robotMask6=regionfill(robotMask5,[627 627+195 627+195 627 627], [15 15 15+190 15+190 15]);
robotMask7=regionfill(robotMask6,[832 832+190 832+190 832 832], [15 15 15+190 15+190 15]);
robotMask8=regionfill(robotMask7,[1038 1038+190 1038+190 1038 1038], [15 15 15+190 15+190 15]);
robotMask9=regionfill(robotMask8,[1247 1247+190 1247+190 1247 1247], [13 13 13+190 13+190 13]);
robotMask10=regionfill(robotMask9,[1455 1455+190 1455+190 1455 1455], [11 11 11+190 11+190 11]);
robotMask11=regionfill(robotMask10,[1663 1663+185 1663+185 1663 1663], [15 15 15+185 15+185 15]);
 
robotMask12=regionfill(robotMask11,[13 13+185 13+185 13 13], [220 220 220+190 220+190 220]);
robotMask13=regionfill(robotMask12,[210 210+190 210+190 210 210], [218 218 218+190 218+190 218]);
robotMask14=regionfill(robotMask13,[419 419+190 419+190 419 419], [220 220 220+190 220+190 220]);
robotMask15=regionfill(robotMask14,[627 627+190 627+190 627 627], [220 220 220+190 220+190 220]);
robotMask16=regionfill(robotMask15,[832 832+190 832+190 832 832], [220 220 220+190 220+190 220]);
robotMask17=regionfill(robotMask16,[1038 1038+190 1038+190 1038 1038], [220 220 220+190 220+190 220]);
robotMask18=regionfill(robotMask17,[1247 1247+190 1247+190 1247 1247], [220 220 220+190 220+190 220]);
robotMask19=regionfill(robotMask18,[1460 1460+190 1460+190 1460 1460], [218 218 218+190 218+190 218]);
robotMask20=regionfill(robotMask19,[1663 1663+185 1663+185 1663 1663], [220 220 220+190 220+190 220]);

robotMask21=regionfill(robotMask20,[13 13+185 13+185 13 13], [428 428 428+185 428+185 428]);
robotMask22=regionfill(robotMask21,[210 210+190 210+190 210 210], [428 428 428+190 428+190 428]);
robotMask23=regionfill(robotMask22,[419 419+190 419+190 419 419], [428 428 428+190 428+190 428]);
robotMask24=regionfill(robotMask23,[627 627+190 627+190 627 627], [428 428 428+186 428+186 428]);
robotMask25=regionfill(robotMask24,[832 832+190 832+190 832 832], [428 428 428+186 428+186 428]);
robotMask26=regionfill(robotMask25,[1038 1038+190 1038+190 1038 1038], [428 428 428+186 428+186 428]);
robotMask27=regionfill(robotMask26,[1247 1247+190 1247+190 1247 1247], [428 428 428+190 428+190 428]);
robotMask28=regionfill(robotMask27,[1457 1457+190 1457+190 1457 1457], [428 428 428+190 428+190 428]);
robotMask29=regionfill(robotMask28,[1663 1663+185 1663+185 1663 1663], [428 428 428+185 428+185 428]);

robotMask30=regionfill(robotMask29,[13 13+185 13+185 13 13], [631 631 631+185 631+185 631]);
robotMask31=regionfill(robotMask30,[210 210+190 210+190 210 210], [631 631 631+190 631+190 631]);
robotMask32=regionfill(robotMask31,[419 419+190 419+190 419 419], [631 631 631+190 631+190 631]);
robotMask33=regionfill(robotMask32,[627 627+195 627+195 627 627], [631 631 631+195 631+195 631]);
robotMask34=regionfill(robotMask33,[832 832+190 832+190 832 832], [631 631 631+190 631+190 631]);
robotMask35=regionfill(robotMask34,[1038 1038+190 1038+190 1038 1038], [631 631 631+190 631+190 631]);
robotMask36=regionfill(robotMask35,[1247 1247+190 1247+190 1247 1247], [631 631 631+190 631+190 631]);
robotMask37=regionfill(robotMask36,[1457 1457+190 1457+190 1457 1457], [631 631 631+190 631+190 631]);
robotMask38=regionfill(robotMask37,[1663 1663+185 1663+185 1663 1663], [631 631 631+185 631+185 631]);

robotMask39=regionfill(robotMask38,[13 13+185 13+185 13 13], [836 836 836+185 836+185 836]);
robotMask40=regionfill(robotMask39,[210 210+190 210+190 210 210], [836 836 836+190 836+190 836]);
robotMask41=regionfill(robotMask40,[419 419+190 419+190 419 419], [836 836 836+190 836+190 836]);
robotMask42=regionfill(robotMask41,[627 627+195 627+195 627 627], [836 836 836+190 836+190 836]);
robotMask43=regionfill(robotMask42,[832 832+190 832+190 832 832], [836 836 836+190 836+190 836]);
robotMask44=regionfill(robotMask43,[1038 1038+190 1038+190 1038 1038], [836 836 836+190 836+190 836]);
robotMask45=regionfill(robotMask44,[1247 1247+190 1247+190 1247 1247], [836 836 836+190 836+190 836]);
robotMask46=regionfill(robotMask45,[1457 1457+190 1457+190 1457 1457], [836 836 836+190 836+190 836]);
robotMask47=regionfill(robotMask46,[1663 1663+185 1663+185 1663 1663], [836 836 836+190 836+190 836]);

%%
% mazeGRAY = rgb2gray(mazeCapture);
se = strel('square',1);
se2 = strel('line',1,2);
se3 = strel('line',2,1);
se4 = strel('rectangle',[5 5]);
se5 = strel('rectangle',[8 8]);

c=robotMask47;
b=[0 1 0; 1 1 1; 0 1 0];
a1=imdilate(c,b);
a2=imerode(c,b);
a3=c-a2;
a4=a1-c;
a5=a1-a2;
a6=imclose(a5,se4);
a6=imfill(a6,4);
a6=imclose(a6,se4);
a7=a6>70;
[centre, radii]=imfindcircles(a7,[6 20]);
figure, imshow(mazeCapture),title('Maze Walls'); hold on;
visboundaries(a7,'Color','r');
rectangle('position',[7 7 1850 1027],'EdgeColor','red','LineWidth',3);
% viscircles(centre,radii,'Color','k','LineWidth',4);

% hold off;
%% 3. Detect the location of the robot and indicate it on the image
figure, imshow(mazeCapture),title('Robot Location'); 
bot=viscircles(botCentre,80,'Color','c');
if (botCentre(1)< 203)
    col = 1;
elseif (botCentre(1)>203 && botCentre(1)<409)
    col = 2;
elseif (botCentre(1)>409 && botCentre(1)<613) 
    col = 3;
elseif (botCentre(1)>613 && botCentre(1)<826)
    col = 4;
elseif (botCentre(1)>826 && botCentre(1) < 1031)
    col = 5;
elseif (botCentre(1)>1031 && botCentre(1) < 1246)
    col = 6;
elseif (botCentre(1)>1246 && botCentre(1) < 1438)
    col = 7;
elseif (botCentre(1)>1438 && botCentre(1) < 1662)
    col = 8;
else
    col = 9;
end

if (botCentre(2)< 207)
    row = 1;
elseif (botCentre(2)>207 && botCentre(2)<412)
    row = 2;
elseif (botCentre(2)>412 && botCentre(2)<620) 
    row = 3;
elseif (botCentre(2)>620 && botCentre(2)<835)
    row = 4;
else
    row = 5;
end
robotPos = ['Robot is in Row ',num2str(row),' and Column ', num2str(col)];
disp(robotPos)
%% 4. Detect the heading of the robot and indicate it on the image
direction=heading(FrontCentre,BackCentre);

if strcmp(direction,'0\n')
    dir=insertText(mazeCapture,[900 500],'NORTH','FontSize',30);
    disp('Robot is heading North');
elseif strcmp(direction,'1\n')
    dir=insertText(mazeCapture,[900 500],'EAST','FontSize',30);
    disp('Robot is heading East')
elseif strcmp(direction,'2\n')
   dir= insertText(mazeCapture,[900 500],'SOUTH','FontSize',30);
    disp('Robot is heading South')
else
   dir= insertText(mazeCapture,[900 500],'WEST','FontSize',30);
    disp('Robot is heading West')
end
imshow(dir);

%% 5. Automatically encode the layout of the maze and the starting location and heading of the robot.
% 1 number for heading
% 2 numbers for row/col
% 2 separate strings or vertical and horizontal walls

se2 = strel('line',20,90);
se3 = strel('line',20,0);
mazeERODE4ARRAY=imdilate(a7,se2);
mazeERODE4ARRAY=imdilate(mazeERODE4ARRAY,se3);
figure;imshow(mazeERODE4ARRAY);title('Eroded Maze');

horizontalArray=zeros(6,9);
horizontalArray(1,:) = 1;
horizontalArray(6,:) = 1;
verticalArray=zeros(5,10);
verticalArray(:,1) = 1;
verticalArray(:,10) = 1;
horSegment=170;
vertSegment=175;
XrowPixel=[23;225;437;639;850;1057;1263;1477;1683;1869];
YrowPixel=[11;212;418;625;829;1029];
XcolPixel= [3; 199; 411; 617; 825; 1031; 1233; 1445; 1653; 1853];
YcolPixel= [37; 239; 447; 655; 861];

%Vertical array
for j=1:5
    for i=2:9
        mazeIMB2=mazeERODE4ARRAY;
        rectangle('position',[XcolPixel(i)-5 YcolPixel(j) 10 vertSegment],'EdgeColor','blue','LineWidth',2);
        rectCoords = [XcolPixel(i)-5 YcolPixel(j) 10 vertSegment]';
        wallseg=imcrop(mazeIMB2,[rectCoords(1) rectCoords(2) rectCoords(3) rectCoords(4)]);
        nzeros=numel(wallseg)-nnz(wallseg);
        if(nzeros<(numel(wallseg)/2))
            verticalArray(j,i)= 1;
        else
            verticalArray(j,i)=0;
        end
    end
end  

%Horizontal array
for j=2:5
    for i=1:9
        mazeIMB2=mazeERODE4ARRAY;
        rectangle('position',[XrowPixel(i) YrowPixel(j)-5 horSegment 10],'EdgeColor','blue','LineWidth',2);
        rectCoords = [XrowPixel(i) YrowPixel(j)-5 horSegment 10]';
        wallseg=imcrop(mazeIMB2,[rectCoords(1) rectCoords(2) rectCoords(3) rectCoords(4)]);
        nzeros=numel(wallseg)-nnz(wallseg);
        if(nzeros<(numel(wallseg)/2))
            horizontalArray(j,i)= 1;
        else
            horizontalArray(j,i)=0;
        end
    end
end    
VertString=mat2str(verticalArray(:)');
VertString(strfind(VertString, ' ')) = [];
VertString(strfind(VertString, '[')) = [];
VertString(strfind(VertString, ']')) = [];
newLine='\n';
VertString=append(VertString,newLine)
HoriString=mat2str(horizontalArray(:)');
HoriString(strfind(HoriString, ' ')) = [];
HoriString(strfind(HoriString, '[')) = [];
HoriString(strfind(HoriString, ']')) = [];
HoriString=append(HoriString,newLine)

%% serial print output for arduino
% %serial.write;
% s=serial('COM4','BAUD',9600);
% fopen(s)
% % fprintf(s,'%c\n',direction);
% fwrite(s,char(direction));
% % idn=fscanf(s);
% fclose(s)
% 
% board = arduino();
% led = 'D13';
% while(1)
%     switch direction
%         case '0\n'
%             disp('North Heading');    
%             writeDigitalPin(board,led,1);    
%             pause(1);
%             writeDigitalPin(board,led,0);     
%             pause(1);
%             writeDigitalPin(board,led,1);    
%             pause(1);
%             writeDigitalPin(board,led,0);     
%             pause(1);
%         case '1\n'
%             disp('East Heading');    
%             writeDigitalPin(board,led,1);    
%             pause(2);
%             writeDigitalPin(board,led,0);     
%             pause(2);
%             writeDigitalPin(board,led,1);    
%             pause(2);
%             writeDigitalPin(board,led,0);     
%             pause(2);
%         case '2\n'
%             disp('North Heading');    
%             writeDigitalPin(board,led,1);    
%             pause(3);
%             writeDigitalPin(board,led,0);     
%             pause(3);
%             writeDigitalPin(board,led,1);    
%             pause(3);
%             writeDigitalPin(board,led,0);     
%             pause(3);
%         case '3\n'
%             disp('East Heading');    
%             writeDigitalPin(board,led,1);    
%             pause(4);
%             writeDigitalPin(board,led,0);     
%             pause(4);
%             writeDigitalPin(board,led,1);    
%             pause(4);
%             writeDigitalPin(board,led,0);     
%             pause(4);
%     end
% end
% disp('close Arduino board');
% clear board;

%%
function poleRemove(centre, radii, image)
kernel=zeros(10,10);
    for i=1:numel(radii)
        for j=1:10
        image(round(centre(i,1)-j),round(centre(i,2)-j))=false;
        image(round(centre(i,1)+j),round(centre(i,2)-j))=false;
        image(round(centre(i,1)-j),round(centre(i,2)+j))=false;
        image(round(centre(i,1)+j),round(centre(i,2)+j))=false;
        end
    end
end

function direction = heading(FrontCentre,BackCentre)
    
    if (abs(FrontCentre(1) - BackCentre(1))<30 && (FrontCentre(2) < BackCentre(2)))
        direction = '0\n';
    elseif ((FrontCentre(1) > BackCentre(1)) && abs(FrontCentre(2) - BackCentre(2))<30)
        direction = '1\n';
    elseif (abs(FrontCentre(1) - BackCentre(1))<30 && (FrontCentre(2) > BackCentre(2)))
        direction = '2\n';
    elseif ((FrontCentre(1) < BackCentre(1)) && abs(FrontCentre(2) - BackCentre(2))<30)
        direction = '3\n';
    end
end

function [blueBW,maskedBlueImage] = blueMask(RGB)

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.610;
% channel1Max = 0.620;
channel1Max = 0.630;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.925;
% channel2Min = 0.942;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.169;
% channel3Min = 0.388;
channel3Max = 0.471;
% channel3Max = 0.224;

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

function [redBW,maskedRedImage] = redMask(RGB)

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.008;
% channel1Max = 0.020;
channel1Max = 0.029;
% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.654;
% channel2Max = 0.725;
channel2Max = 0.794;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.353;
% channel3Min = 0.592;
% channel3Min = 0.600;
% channel3Max = 0.439;
channel3Max = 0.639;
% channel3Max = 0.643;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
redBW = sliderBW;

% Initialize output masked image based on input image.
maskedRedImage = RGB;

% Set background pixels where BW is false to zero.
maskedRedImage(repmat(~redBW,[1 1 3])) = 0;

end