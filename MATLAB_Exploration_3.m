%clear workspace variables, command window, and close figures
clc; clear; close all;
%set the window style to normal
set(0,'DefaultFigureWindowStyle','normal')

%read in image of the course 
%this image file must be in the same folder as the script
img = imread('Exploration_03_Course_Image.png');

%set course axes ranges
x_min = 0;
x_max = 72;
y_min = 0;
y_max = 36;

%open MATLAB figure with course image at set location on screen
hFig = figure(1);
set(hFig, 'Position', [150 150 879 478])
imagesc([x_min x_max], [y_min y_max], flipud(fliplr(img)));
set(gca,'xdir','reverse')
set(gca,'ydir','normal')
set(gca, 'YAxisLocation','right')
set(hFig, 'Resize','off')
hold on

%read in logged RPS data; change this name as necessary
Path = csvread('LOG077.TXT');

%ADD CODE TO FILTER -1's AND -2's AND PLOT HERE
t=1;
while(t<=length(Path))
    if (Path(t,1)<0)
        Path(t,:)=[];
    else
        plot(Path(t,2),Path(t,1),'ro');
        t=t+1;
        
    end
end
plot(Path(1:end,2),Path(1:end,1),'r-');
