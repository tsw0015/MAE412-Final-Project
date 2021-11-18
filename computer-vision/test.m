clear, clc, close all

filter_threshold = 0.43;

%assing camera to the port the webcam is (use webcamlist command in command
%window to list out the difference webcam devices connected to your device
%and assign accordingly
cam = webcam('/dev/video0');

%continually update the picture using while loop (ctrl + C in command
%window to kill the process)
while(true)
    %take an image
    img = snapshot(cam);
%     img = imread("rtwister_shop_thumb.png");

    %split into individual Red Green and Blue Matricies
    [R,G,B] = imsplit(img);
    R = double(R);
    G = double(G);
    B = double(B);

    %create mask for filtering white
    white_filter = uint16(R)+uint16(G)+uint16(B);
    white_filter = double(white_filter);

    %Find percentages of what color each pizel is made up of
    R_filter = R./white_filter;
    B_filter = B./white_filter;
    G_filter = G./white_filter;

    %Use the thresholding to mask the image
    R_filter = R_filter > filter_threshold;
    G_filter = G_filter > filter_threshold;
    B_filter = B_filter > filter_threshold;

    %create new image
    new_image(:,:,1) = uint16(R).*uint16(R_filter);
    new_image(:,:,2) = uint16(G).*uint16(G_filter);
    new_image(:,:,3) = uint16(B).*uint16(B_filter);
    new_image = uint8(new_image);

    %circle detection
    imshow(img)
    [centers,radii] = imfindcircles(new_image,[8 100],'Sensitivity',0.92);
    viscircles(centers,radii);    
    
end