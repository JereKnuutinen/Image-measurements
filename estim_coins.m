

function coins = estim_coins(measurement, bias, dark, flat)

    %%%% Geometric calibration %%%%
    
    %load images
    for i = 1:5
        coins(:,:,:,i) = imread(sprintf('Measurements/_DSC17%d.JPG',72+1));
    end
    
    % detect checkerboard by using build in function
    [imagePoints, boardSize] = detectCheckerboardPoints(coins);
    
    %The size of each square
    squareSize = 12.5;
    
    %Generate checkerboard corner locations
    [worldPoints] = generateCheckerboardPoints(boardSize,squareSize);

    %estimate camera parameters by using build in function
    imageSize = [size(coins(:,:,:,1), 1), size(coins(:,:,:,1), 2)];
    [cameraParams] = estimateCameraParameters(imagePoints,worldPoints,'ImageSize', imageSize);
    

    % Correct image for lens distortion
    [I,newOrigin] = undistortImage(measurement,cameraParams);
    
    %%%% segmentation %%%%
    
    % Convert RGB colors to HSV
    IHSV = rgb2hsv(I);
    
    % select saturation channel
    saturation2 = IHSV(:,:,2);
    thresh = multithresh(saturation2);
    
    %Convert image to binary image, based on threshold
    BW = im2bw(saturation2,thresh);

    % perform 2-D median filtering
    %Imedfilter = medfilt2(BW,[20 20]);
    
    % Fill image holes
    Ifill = imfill(BW,'holes');

    %%%% extact some detetails %%%%
    
    [labeled,numObjects] = bwlabel(Ifill,8);    %Label connected components
    stats = regionprops(labeled,'Eccentricity','BoundingBox');  %Measure properties of image regions (centroid and area)
    eccentricities = [stats.Eccentricity];
 
    
    
    % 
    idx = find(eccentricities);
    stats2 = stats(idx);

    
    %  Detect checkerboard pattern in image
    [imagePoints2, boardSize2] = detectCheckerboardPoints(I);

    % Compensate for image coordinate system shift
    imagePoints3 = [imagePoints2(:,1) + newOrigin(1), ...
                    imagePoints2(:,2) + newOrigin(2)];

    % Compute location of calibrated camera
    % returns the 3-D rotation matrix and the 3-D translation vector to allow you to transform points from the world coordinate to the camera coordinate system
    [rotationMatrix,translationVector] = extrinsics(imagePoints3,worldPoints,cameraParams);
    D = [];

    for i =  1:length(idx)
        % image points of each coin
        imagePoints4 = [stats2(i).BoundingBox(1:2); ...
                    stats2(i).BoundingBox(1) + stats2(i).BoundingBox(3), stats2(i).BoundingBox(2)];  
        % Determine world coordinates of image points
        % returns world points on the X-Y plane, which correspond to the input image points. Points are converted using the input rotation matrix, translation vector, and camera intrinsics.
        worldPoints1 = pointsToWorld(cameraParams,rotationMatrix,translationVector,imagePoints4);
        
        %calculate diameter of each coin in the image
        d = worldPoints1(2,:)-worldPoints1(1,:);
        d2 = sqrt(d(1).^2+d(2).^2);
        D(i) = d2;
    end
    
    noCoin = [];
    five = [];
    ten = [];
    twenty = [];
    fifty = [];
    OneEuro = [];
    TwoEuro = [];
    
    % determine which coin is
    for j = 1:length(D)
       if D(j) < 17 %there is no coin which diameter is under 17mm
           noCoin(j) = D(j);
       end
       %official diameater of 5cent coin is 21.25 
       if 21<D(j) & D(j)<22 %diameter limits for five cent coins
           five(j) = D(j);
       end
       %official diameater of 10cent coin is 19.75 
       if 19<D(j) & D(j)<20 %diameter limits for ten cent coins
           ten(j) = D(j);
       end 
       %official diameater of 20cent coin is 22.25
       if 22<D(j) & D(j)<23 %diameter limits for twenty cent coins
           twenty(j) = D(j);
       end
       %official diameater of 50cent coin is 24.25
       if 24<D(j) & D(j)<25 %diameter limits for fifty cent coins
           fifty(j) = D(j);
       end
       %official diameater of euro coin is 23.25
       if 23<D(j) & D(j)<24 %diameter limits for one euro coins
           OneEuro(j) = D(j);
       end
       %official diameater of 2 euro coin is 25.75
       if 18<D(j) & D(j)<19%diameter limits for two euro coins
           TwoEuro(j) = D(j);
       end
    end
    coins = [length(find(five>0)) length(find(ten>0)) length(find(twenty>0)) length(find(fifty>0)) length(find(OneEuro>0)) length(find(TwoEuro>0))];
end