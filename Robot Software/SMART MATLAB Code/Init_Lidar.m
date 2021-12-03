function [S_Lidar] = Init_Lidar(PortNumber)
 
port = strcat('/dev/ttyUSB',num2str(PortNumber) );
 
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

S_Lidar = serial(port,'baudrate',230400);            % Define serial port

set(S_Lidar,'Timeout',0.1);
set(S_Lidar,'InputBufferSize',4248);
set(S_Lidar,'Terminator','CR');
fopen(S_Lidar);

%% 
pause(0.1);
fprintf(S_Lidar,'SCIP2.0');
pause(0.3);
fscanf(S_Lidar)

fprintf(S_Lidar,'VV');
pause(0.3);
fscanf(S_Lidar)

fprintf(S_Lidar,'BM');
pause(0.3);
fscanf(S_Lidar)





end
