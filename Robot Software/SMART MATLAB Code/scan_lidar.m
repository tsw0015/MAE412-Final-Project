%%%%%%%% Turtlebot HLDS MATLAB Interface %%%%%

% Author: Christopher Tatsch

%Require CP210x USB to UART Bridge VCP Drivers which may be found in the
%following link
%(https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers)

%function [x, y, z] =scan_lidar(port)
function ranges =scan_lidar(s)


    %variables
    get_scan = false;
    ranges = zeros(360,1);
    intensities = zeros(360,1);
    raw_data = zeros(2520,1);


    while get_scan == false
        raw_data(1:2) = fread(s,2,'uint8'); %%get_start_sequence_data
        raw_data = uint8(raw_data);
        if raw_data(1)==250 & raw_data(2)==160
            get_scan = true;
            raw_data(3:2520) = fread(s,2518,'uint8');
            index=0;
            for i = 1:60 %%Read data in pairs of 6
                if raw_data((i-1)*42+1)==250 & raw_data((i-1)*42+2)==160+(i-1)

                   for j=0:5
                       intensity = typecast(uint8([raw_data(42*(i-1)+6+j*6) raw_data(42*(i-1)+5+j*4)]),'uint16');
                       range = typecast(uint8([raw_data(42*(i-1)+8+j*6) raw_data(42*(i-1)+7+j*6)]),'uint16')/1000.0;
                       intensities(360-index)=intensity;
                       ranges(360-index)=range;
                       index=index+1;
                   end
                end
            end 
        end
    end

end








