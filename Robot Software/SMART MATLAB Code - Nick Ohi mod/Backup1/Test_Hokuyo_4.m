%-----------------------------------------------------------------------%
%   Test Hokuyu URG-04LX 2D LIDAR                                       %
%   Modified from Ehab Al Khatib's code                                 %
%-----------------------------------------------------------------------%
%%
clear all
close all
clc

S_Hokuyo=Init_Hokuyo(7);    % initalize the Lidar
pause(0.1);
fscanf(S_Hokuyo)

rangescan=zeros(1, 682);            
angles=(-120:240/682:120-240/682)*pi/180;   % an Array of all the scan angles
fprintf(S_Hokuyo,'GD0044072500');           % request a scan
pause(0.1);

tic
while (1)
    
    rangescan = Read_Hokuyo(S_Hokuyo, rangescan);
    
    x=rangescan.*cos(angles);
    y=rangescan.*sin(angles);
    plot(x,y,'.')
    axis([-5000 5000 -5000 5000]);
    drawnow
    
    flushinput(S_Hokuyo);
    fprintf(S_Hokuyo,'GD0044072500');           % request a scan
    toc
    pause(0.1)
end
% 
  
% Switches off Laser and Closes Serial Comm Link
fprintf(S_Hokuyo,'QT');
fclose(S_Hokuyo);
%close all;


