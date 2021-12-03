%-----------------------------------------------------------------------%
%   Initialize Hokuyu URG-04LX 2D LIDAR                                 %
%   Modified from Ehab Al Khatib's code                                 %
%-----------------------------------------------------------------------%


function [S_Hokuyo] = Init_Hokuyo(ComPortNumber)
 
port = strcat('COM',num2str(ComPortNumber) );
 
out = instrfind('Port', port);  % Check to see if THAT serial port is already defined in MATLAB
if (~isempty(out))  % It is
    disp('WARNING:  port in use.  Closing.')
    if (~strcmp(get(out(1), 'Status'),'open'))  % Is it open?
        delete(out(1)); % If not, delete
    else  % is open
        fclose(out(1));
        delete(out(1)); 
    end
end

S_Hokuyo = serial(port,'baudrate',115200);            % Define serial port

set(S_Hokuyo,'Timeout',0.1);
set(S_Hokuyo,'InputBufferSize',4248);
set(S_Hokuyo,'Terminator','CR');
fopen(S_Hokuyo);

%% 
pause(0.1);
fprintf(S_Hokuyo,'SCIP2.0');
pause(0.3);
fscanf(S_Hokuyo)

fprintf(S_Hokuyo,'VV');
pause(0.3);
fscanf(S_Hokuyo)

fprintf(S_Hokuyo,'BM');
pause(0.3);
fscanf(S_Hokuyo)
% fprintf(S_Hokuyo,'GD0044072500');

end


% pause(0.1);
% fscanf(S_Hokuyo)
% %%
% Scan_GUI