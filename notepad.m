function [xdir_out2,ydir_out,zdir_out, filteredImage_, dirHis, dir, trackEndChecker_in, turnCheck_in, circleApproachCheck_in, centreCheck_in] = fcn(R, G, B, dirHis_out, trackEndChecker_, turnCheck_, dir_out, circleApproachCheck_, centreCheck_ )
xdir_out2=0;ydir_out=0;zdir_out=0;
%% UPDATED CODE
%% filter the image !!TO BE PLACED IN THE IMAGE PROCESSING SECTION
trackColor=[ 255 0 255]; 
filteredImage_ = imageFilter(trackColor,R,B,G);

%% Stage 1
%entry:
% global initialise
% if isempty(initialise)
%     initialise=1;
% end


%initialize
h_=50;
increment_=0.3;


%Stage 2 variables
% global dirHis_ trackEndChecker_ turnCheck_ dir_ circleApproachCheck_ centreCheck_ coneCoords_ trackEndTrue_ trackEndFalse_

if ~dirHis_out %initialises the global variables!
    dirHis_out = 0;trackEndChecker_=0;turnCheck_=0;dir_out=0;circleApproachCheck_=0;centreCheck_=0;
    coneCoords_ = coneCreate(h_,increment_,filteredImage_); %cone 
    [trackEndTrue_,trackEndFalse_] = trackEndDetectFilterCreate(h_,coneCoords_); %trackEndDetector!
    
zValue_=-1.1;%rise to altitude
pause(1000);
initialise =0;
end
coneCoords_=0;

if ~coneCoords_
    error("coneCoords empty!!!!!")
end

%%%%%%%%%%%%%%%%%%
% S_ = coneCreate(h_,increment_,I2_); %cone
% [trackEndTrue_,trackEndFalse_] = trackEndDetectFilterCreate(h_,S_); %trackEndDetector!
% %Stage 2 variables
% %used to store previous values of dir
% tHis_=tHisInput;
% trackEndChecker_= trackEndCheckerInput;
% turnCheck_=turnCheckInput;
% dir_=dirInput;
% circleApproachCheck_=circleApproachCheckInput;%stage 3
% centreCheck_=centreCheckInput;
%%%%%%%%%%%%%%%%%%

%% Stage 2

 
 if ~initialise %used in place of zValeRef<-1
     
   
     
 %entry:
TrackEndFalseSum_=TrackEndRegionSum(trackEndFalse_,filteredImage_,dir_out);
if (TrackEndFalseSum_/255) <52
    TrackEndTrueSum_=TrackEndRegionSum(trackEndTrue_,filteredImage_,dir_out);
    if (TrackEndTrueSum_/255)>100
        trackEndChecker_=1;
    end
end


%during:
if ~trackEndChecker_
    if turnCheck_
        %function returns true if turning, if straight path, then it calls straightPath func, which runs until it "turns"
        [check_, dirNew_] = turn(filteredImage_,coneCoords_, dir_out, dirHis_out);
        if ~dirHis_out %this pathway stores the first nonzero value of tHis %initialise
            dirHis_out=dirNew_;
            turnCheck_=check_;
            
            a_=0.0005;
            %moves drone forward in direction tHis, so we can repeat scan to determine if straight path
            xValue_=a_*cos(dirHis_out);
            yValue_=a_*sin(dirHis_out);
        else
            dirHis_out=dirNew_;
            turnCheck_=check_;
        end
    end
end




 end
    
% stage 3

if ~initialise
    if trackEndChecker_
    
    
        
% during:
if circleApproachCheck_==0
    [ctrls_1,check_] = trackEndDroneEnteringCircle(trackEndFalse_,filteredImage_,dir_out); %[ctrls,check] = trackEndDroneEnteringCircle(trackEndFalse,filteredImage,dirHis)
    xValue_=ctrls_1(1);yValue_=ctrls_1(2);
    circleApproachCheck_=check_;
else
    centreChecksum=sum(centreCheck_,'all') - 1;
    if ~centreChecksum
        [ctrls_2, check_] = endCircleScan(coneCoords_,filteredImage_);
        centreCheck_=check_;
        xValue_=ctrls_2(1);yValue_=ctrls_2(2);
    else
        zValue_=-0.9;
            [ctrls_3, check_] = endCircleScan(coneCoords_,filteredImage_); %[ctrls, check] = endCircleScan(coneCoords,filteredImage)
            centreCheck_=check_;
             centreChecksum=sum(centreCheck_,'all') - 1;
            
        if ~centreChecksum
            xValue_=ctrls_3(1);yValue_=ctrls_3(2);
        else
            zValue_=0;
            pause(5000)
            error("Ladies and Gentlemen, you have landed");
        end
    end
end



    end
end

xdir_out2=0;%xValue_;
ydir_out=yValue_;
zdir_out=zValue_;
dirHis=dirHis_out;
dir=dir_out;
trackEndChecker_in=trackEndChecker_;
turnCheck_in=turnCheck_;
circleApproachCheck_in=circleApproachCheck_;
centreCheck_in=centreCheck_;


    %% functions
function [filteredImage] = imageFilter(trackColor,R,B,G)%%%%%%%%%%%%%%%%%%
 C = trackColor;

% n is the threshold of dominance, intensity above 150/255 is considered
% dominant out of R G and B
n = C < 150;

% whichever is recessive (R G B or any combination of the 3) is made
% negative so the intensities of required colour stands out
C = n.*C*-2 + C;

% colour is normalised, 310 works better than 255 to accomodate colours too
% close to any one value of 255, seems to work better
Cr = C/310;

%video feed is combined of varying intensities based on colour
%thresholding 
I = Cr(1)*R + Cr(2)*G + Cr(3)*B; %INPUT

%intensities are further thresholded to drown out noise, any intensities
%below number is eliminated
I = I > 180; %only parts on the path are highlighted

% visualisation tool
% I2 is a b&W image feed with only the track highlighted. 
%255 --> white

I2 = zeros(size(I));
I2(I) = 255;  %or I2=255.*I;

filteredImage=I2; % OUTPUT
end

function[coneCoords] = coneCreate(yh,increment,filteredImage) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
y_range = 0:yh; %height

% pre allocating a matrix for the cone x and y coordinate positions
coneCoords = ones(919, 2);

% counters
m=1;
c=1; % used to reference spots within matrix S, used to store values of the cone.
n=1; %used to determine the range of x in each lattice of the cone %used to determine the increase in x range as you jump from lattice to lattice

for y = y_range
    
    for x = -n:n

        coneCoords(c,1) = x;
        coneCoords(c,2) = y;
        c=c+1;
    end
    %this line is used to indirectly adjust the cone angle, the larger the
    %increment to m, the larger the cone value as it lenghtens the range of
    %rows each layer of the cone. imagine the cone as being built from the
    %point at the origin to the widest part in the positive y direction
    m = m+increment;
    n=round(m);
end
    
%centering the cone
[my,mx] = size(filteredImage);

% we have created the cone in the cartesian reference with 0,0 origin
% reference, however matlab matrices start indices at 1,1 at the top left
% increasing towards bottom right. hence to adjust, we must find the
% relative centre of the image matrix

mx_c = round(0.5*mx);
my_c = round(0.5*my);   


% centring and creating the cone
coneCoords = coneCoords'; % from [x y] --> [x;y]

x_range = coneCoords(1,:);
y_range = coneCoords(2,:);

mx_range = x_range +  mx_c;
my_range = my_c - y_range;

coneCoords=[mx_range;my_range]; %OUTPUT
end

function [trackEndTrue,trackEndFalse] = trackEndDetectFilterCreate(yh,coneCoords)
sections=3;
yrange=linspace(0,yh,sections+1);

trackEndTrue=zeros(2, length(coneCoords));
trackEndFalse=zeros(2, length(coneCoords));

x=coneCoords(1,:);
y=coneCoords(2,:);

for i=1:length(coneCoords)
    if coneCoords(2,i)>round(yrange(end-1))
        trackEndTrue(1,i)=x(i); %OUTPUT
        trackEndTrue(2,i)=y(i); %OUTPUT
    end
    
    if(coneCoords(2,i))< round(y(end-1)) && coneCoords(2,i)>round(yrange(end-2))
            trackEndFalse(1,i)=x(i);
            trackEndFalse(2,i)=y(i);
    end
end
end

function [check, dirHis] = turn(filteredImage,coneCoords, dir, dirHis)
%check = 1, if turning!!

if dirHis   
[t1] = scan(dir, coneCoords,pi()/2,filteredImage); %scans the region, it transforms the cone (rotation)

    if abs(t1-dirHis)<0.1*dirHis %checks if two directions taken consecutively point in same directio --> straight path if true
         straightPathCheck=straightPath(dir,coneCoords,filteredImage);
         check=0;
    else
        check=1; 
    end

else %when tHis is empty, we will scan and output tHis
    dirHis = scan(dir, coneCoords,pi()/2,filteredImage);
    check=1;
    
end
end
    
function [dirOut] = scan(dir, coneCoords,phi,filteredImage)
% [tout] = scan(t_, S,pi()/2)
%S=[x;y]
% given input t, we scan a region t-pi/2 to t+pi/2 for the cone area 
% and corresponding t that has most of the path on it

thetaRange=[dir-phi:phi/6:dir+phi];
main_g=zeros(size(coneCoords));

[mx, my]=size(coneCoords);
mx_c = mx/2; my_c = my/2;


dirSumCurr=zeros(1,length(thetaRange));

for i=1:length(thetaRange)
    dir=thetaRange(i);
    A = [cos(dir) sin(dir) ; -sin(dir) cos(dir)]*[coneCoords];
    
    x_range = round(A(1,:));
    y_range = round(A(2,:));
    
    mx_range = x_range +  mx_c;
    my_range = my_c - y_range;
    
    
    
        % this counts the number of pixels highlighted in the cone, when it
        % meets a certain threshold, the direction is set

        for j = 1:length(my_range)
            main_g(my_range(j),mx_range(j)) = filteredImage(my_range(j),mx_range(j));
            
        end
        
        dirSumCurr(i)=sum(main_g,'all');
end

%% determines t which will have max sum value in the range we scanned for
[val, pos] = max(dirSumCurr);
dirOut=thetaRange(pos);
end

function check = straightPath(dir,coneCoords,filteredImage)
%check =1 if on straight path
trpseMatrix=[cos(dir) sin(dir) ; -sin(dir) cos(dir)];
S_=trpseMatrix*coneCoords; %cone points in direction, dir

x_range = round(S_(1,:));
y_range = round(S_(2,:));

[my,mx] = size(filteredImage);

mx_c = round(0.5*mx);
my_c = round(0.5*my);   

mx_range = x_range +  mx_c;
my_range = my_c - y_range; %cone has been placed in the middle


main_g=zeros(size(filteredImage));

for i = 1:length(my_range)
    main_g(my_range(i),mx_range(i)) = filteredImage(my_range(i),mx_range(i));
end

sumInt=sum(main_g,'all');

check=1;
while check
   
for i = 1:length(my_range)
    main_g(my_range(i),mx_range(i)) = filteredImage(my_range(i),mx_range(i));
end

sumVal=sum(main_g,'all');

a=0.1;%threshold for sumVal

if sumVal>sumInt*(1+a)||sumVal<sumInt*(1-a) %determines if path is straight
    check=0;
    return
end
end
end

function [sumOut] = TrackEndRegionSum(RegionCoords,filteredImage,dir)
C=RegionCoords;

[mx, my]=size(filteredImage);
main_g=zeros(mx,my); %screen
mx_c = mx/2; my_c = my/2; %used to centre the RegionCoords

  A = [cos(dir) sin(dir) ; -sin(dir) cos(dir)]*[C];%transform/rotation of the RegionCoords
  x_range = round(A(1,:));
  y_range = round(A(2,:));

  mx_range = x_range +  mx_c;%centring of RegionCoords
  my_range = my_c - y_range;

for i=1:length(C)
   main_g(my_range(i),mx_range(i)) = filteredImage(my_range(i),mx_range(i));  
end

sumOut=sum(main_g,"all");
end

function [ctrls,check] = trackEndDroneEnteringCircle(trackEndFalse,filteredImage,dirHis)
dir=dirHis;

screen=zeros(size(filteredImage));

 A = [cos(dir) sin(dir) ; -sin(dir) cos(dir)]*trackEndFalse;
    
    x_range = round(A(1,:));
    y_range = round(A(2,:));
    
[my,mx] = size(filteredImage);

mx_c = round(0.5*mx);
my_c = round(0.5*my);  
    
    mx_range = x_range +  mx_c;
    my_range = my_c - y_range;
    
        for j = 1:length(my_range)
            screen(my_range(j),mx_range(j)) = filteredImage(my_range(j),mx_range(j));
        end
        segmentSum=sum(screen,'all');
        
        if segmentSum/255>20
            %you have centred the mini drone--> adjust if necessary
            check = 1;
            ctrls=[0 0];
        else
            a=1;
            xValue=a.*cos(dirHis);
            yValue=a.*sin(dirHis);
            ctrls=[xValue yValue];
            check=0;
        end
end

function [dirOut] = findFinerDirection(dirHis,filteredImage)
ddc=0;
h=50;
iter=2;
while ~ddc %direction differentiation check!!
    increment=increment/iter;
    phi=pi()/2*1/iter; %change/confirm this!!!
    
    S  = coneCreate(h,increment,filteredImage);
    dirOut  = scan(S,filteredImage,dirHis,phi);

if abs(dirOut-dirHis)<0.1*dirHis/iter %means not significant difference between tout and tHis.
else
    ddc=1; %significant difference between tout and tHis
end
end
end

function [ctrls, check] = endCircleScan(coneCoords,filteredImage)

%initialise
check=0;
ctrls=[0 0];

[mx, my]=size(coneCoords);
mx_c = mx/2; my_c = my/2;
main_g=zeros(size(coneCoords));

%% topSeg
  topSeg=main_g;
  dir=0;
  A = [cos(dir) sin(dir) ; -sin(dir) cos(dir)]*coneCoords;
    
    x_range = round(A(1,:));
    y_range = round(A(2,:));
    
    mx_range = x_range +  mx_c;
    my_range = my_c - y_range;
    
        for j = 1:length(my_range)
            topSeg(my_range(j),mx_range(j)) = filteredImage(my_range(j),mx_range(j));
            
        end
        topSeg_=sum(topSeg,"all");
        
        
  %% bottomSeg
  
  bottomSeg = main_g;
  dir=pi();
  A = [cos(dir) sin(dir) ; -sin(dir) cos(dir)]*coneCoords;
    
    x_range = round(A(1,:));
    y_range = round(A(2,:));
    
    mx_range = x_range +  mx_c;
    my_range = my_c - y_range;
    
        for j = 1:length(my_range)
            bottomSeg(my_range(j),mx_range(j)) = filteredImage(my_range(j),mx_range(j));
            
        end
        bottomSeg_=sum(bottomSeg,"all");
        
        
%% leftSeg

  leftSeg = main_g;
  dir=pi()/2;
  A = [cos(dir) sin(dir) ; -sin(dir) cos(dir)]*coneCoords;
    
    x_range = round(A(1,:));
    y_range = round(A(2,:));
    
    mx_range = x_range +  mx_c;
    my_range = my_c - y_range;
    
        for j = 1:length(my_range)
            leftSeg(my_range(j),mx_range(j)) = filteredImage(my_range(j),mx_range(j));
            
        end
        leftSeg_=sum(leftSeg,"all");
        
 
%% rightSeg
  rightSeg=main_g;
  dir=-pi()/2;
  A = [cos(dir) sin(dir) ; -sin(dir) cos(dir)]*coneCoords;
    
    x_range = round(A(1,:));
    y_range = round(A(2,:));
    
    mx_range = x_range +  mx_c;
    my_range = my_c - y_range;
    
        for j = 1:length(my_range)
            rightSeg(my_range(j),mx_range(j)) = filteredImage(my_range(j),mx_range(j));
            
        end
        
        rightSeg_=sum(rightSeg,"all");
        
%% scan and adjust position
xF=2;
yF=2;

%up and down
if topSeg_>1.1*bottomSeg_ %drone is placed lower compared to the landing circle
    xdir=xF*abs(topSeg_-bottomSeg_)*cos(0);
    x_centred=0;
elseif topSeg_<0.9*bottomSeg_
        xdir=-xF*abs(topSeg_-bottomSeg_)*cos(0);
        x_centred=0;
else
    xdir=0;
    x_centred=1;
end


%left and right
if rightSeg_>1.1*leftSeg_ %drone is placed lower compared to the landing circle
    ydir=yF*abs(rightSeg_-leftSeg_)*cos(pi()/2);
    y_centred=0;
elseif topSeg_<0.9*bottomSeg_
        ydir=-yF*abs(rightSeg_-leftSeg_)*cos(pi()/2);
        y_centred=0;
else
    ydir=0;
    y_centred=1;
end

ctrls=[xdir, ydir];
% check=[x_centred, y_centred ];

if x_centred==1 && y_centred==1 %centered!
    check=1;
else %not yet centered
    check =0;
end
end

end


%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%