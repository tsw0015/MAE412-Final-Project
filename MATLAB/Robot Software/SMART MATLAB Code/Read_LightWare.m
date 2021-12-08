%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read the Lightware SF02/F Laser Rangefinder                       %
% Author: Yu Gu                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Laser_RF] = Read_LightWare(S_LightWare,Laser_RF)

%Laser_RF=0;
LoggerBytesAvailable=get(S_LightWare, 'BytesAvailable');
if (LoggerBytesAvailable>=6)
%     LoggerBytesAvailable
%     return
% else
    SerialData=fscanf(S_LightWare);
    Laser_RF=1000*str2double(SerialData(1:(end-3)));
    if isnan(Laser_RF)
        Laser_RF=0;
    end
end


end