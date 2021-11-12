clear, clc, close all

cam = webcam('/dev/video2');
%preview(cam)
while(true)
    img = snapshot(cam);
    % imshow(img)
    % figure;

    [R,G,B] = imsplit(img);
    R = double(R);
    G = double(G);
    B = double(B);
    filter_threshold = 0.43;

    % imshow(cat(3, R, zeros(size(G)), zeros(size(B))));
    % figure;
    %
    % imshow(cat(3, zeros(size(R)), G, zeros(size(B))));
    % figure;
    %
    % imshow(cat(3, zeros(size(R)), zeros(size(G)), B));
    % figure;
   

    white_filter = uint16(R)+uint16(G)+uint16(B);
    white_filter = double(white_filter);
    % for i = 1:size(white_filter,1)
    %     for j = 1:size(white_filter,2)
    %         if white_filter(i,j) > 500
    %             white_filter(i,j)= 0
    %         end
    %     end clear('cam')
    % end
    % white_filter = white_filter < 375;
    % white_filter = uint8(white_filter);
    R_filter = R./white_filter;
    B_filter = B./white_filter;
    G_filter = G./white_filter;

    R_filter = R_filter > filter_threshold;
    G_filter = G_filter > filter_threshold;
    B_filter = B_filter > filter_threshold;


    new_image(:,:,1) = uint16(R).*uint16(R_filter);
    new_image(:,:,2) = uint16(G).*uint16(G_filter);
    new_image(:,:,3) = uint16(B).*uint16(B_filter);

    imshow(uint8(new_image))
end